package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.RobotState;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.BLUE_TARGET_POSITION;
import static frc.robot.Constants.ShootingConstants.DISTANCE_TO_TIME_OF_FLIGHT_MAP;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_FORWARD;
import static frc.robot.Constants.ShootingConstants.POST_SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.RED_TARGET_POSITION;
import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
import static frc.robot.Constants.Swerve.MAX_SPEED;

public class TeleopShooting extends SequentialCommandGroup {
    public TeleopShooting(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        addCommands(
                new TeleopAim(drivetrain, shooter, translationSup, strafeSup),
                new TeleopThrow(drivetrain, shooter, translationSup, strafeSup).withTimeout(SHOOTING_DELAY + POST_SHOOTING_DELAY)
        );

        SmartDashboard.putNumber("calibration/angle [deg]", 0);
        SmartDashboard.putNumber("calibration/rpm [RPM]", 0);
    }

    private static class TeleopAim extends Command {
        private final DrivetrainSubsystem drivetrain;
        private final ShooterSubsystem shooter;
        private final DoubleSupplier translationSup, strafeSup;

        private final Timer deadlineTimer = new Timer();

        private double virtualTargetDistance = 0;

        public TeleopAim(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
            this.drivetrain = drivetrain;
            this.shooter = shooter;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;

            addRequirements(drivetrain, shooter);
        }

        @Override
        public void initialize() {
            deadlineTimer.restart();

            // Predict the position the robot will be in when the NOTE is released
            RobotState predictedState = RobotState.predict(drivetrain.getPoseHistory(), Timer.getFPGATimestamp() + SHOOTING_DELAY);
            Translation2d targetDisplacement = calculateTargetDisplacement(predictedState);      
            Translation2d virtualTargetDisplacement = calculateVirtualTargetDisplacement(
                targetDisplacement.getNorm(), targetDisplacement, predictedState.getVelocity());
            virtualTargetDistance = virtualTargetDisplacement.getNorm();

        }

        /**
         * Find the displacement to the virtual target, i.e., the target to which the robot should aim s.t.
         * the NOTE enters the actual target, even whilst moving.
         */
        private static Translation2d calculateVirtualTargetDisplacement(double virtualTargetDistance,
                                                                        Translation2d targetDisplacement,
                                                                        ChassisSpeeds velocity) {
            double timeOfFlight = DISTANCE_TO_TIME_OF_FLIGHT_MAP.get(virtualTargetDistance);

            Translation2d displacementDueToRobotVelocity = new Translation2d(
                    velocity.vxMetersPerSecond * timeOfFlight,
                    velocity.vyMetersPerSecond * timeOfFlight
            );

            return targetDisplacement.minus(displacementDueToRobotVelocity);
        }

        /** Calculate the displacement from the centre of the robot to the target */
        private static Translation2d calculateTargetDisplacement(RobotState robotState) {
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            Translation2d targetPosition = alliance == DriverStation.Alliance.Red ? RED_TARGET_POSITION : BLUE_TARGET_POSITION;
            return targetPosition.minus(robotState.getPose().getTranslation());
        }

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            // Predict the position the robot will be in when the NOTE is released
            RobotState predictedState = RobotState.predict(drivetrain.getPoseHistory(), Timer.getFPGATimestamp() + SHOOTING_DELAY);
            Translation2d targetDisplacement = calculateTargetDisplacement(predictedState);      

            // Calculate the displacement of the virtual target, to which the robot so it can
            // score whilst moving
            Translation2d virtualTargetDisplacement = calculateVirtualTargetDisplacement(
                virtualTargetDistance, targetDisplacement, predictedState.getVelocity());
            virtualTargetDistance = virtualTargetDisplacement.getNorm();
            
            SmartDashboard.putNumber("shooting/distance from target [meters]", targetDisplacement.getNorm());
            SmartDashboard.putNumber("shooting/distance from virtual target [meters]", virtualTargetDistance);

            // Aim the azimuth to the virtual target
            drivetrain.driveWithAzimuth(new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED),
                                    virtualTargetDisplacement.getAngle());

            // Aim the shooter
            shooter.setReference(
//                    DISTANCE_TO_REFERENCE_MAP.get(virtualTargetDistance)
                    new ShooterSubsystem.Reference(
                            Rotation2d.fromDegrees(SmartDashboard.getNumber("calibration/angle [deg]", 0)),
                            RPM.of(SmartDashboard.getNumber("calibration/rpm [RPM]", 0))
                    )
            );

//            shooter.setPitchGoal(Rotation2d.fromDegrees(SmartDashboard.getNumber("calibration/angle [deg]", 0)));
        }

        @Override
        public boolean isFinished() {
            boolean flywheelsReady = shooter.flywheelsAtReference();  // TODO for debugging, remove
            boolean pitchReady = shooter.pitchAtReference();  // TODO for debugging, remove
            boolean azimuthReady = drivetrain.azimuthAtGoal();
            boolean reachedDeadline = deadlineTimer.hasElapsed(2.5);

            boolean readyToKick = shooter.atReference() && azimuthReady;

            SmartDashboard.putBooleanArray("flywheels | pitch | azimuth", new boolean[]{flywheelsReady, pitchReady, azimuthReady});

            return readyToKick || reachedDeadline;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                shooter.reset();
            }
        }
    }

    public static class TeleopThrow extends Command {
        private final DrivetrainSubsystem drivetrain;
        private final ShooterSubsystem shooter;
        private final DoubleSupplier translationSup, strafeSup;

        public TeleopThrow(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter,
                           DoubleSupplier translationSup, DoubleSupplier strafeSup) {
            this.drivetrain = drivetrain;
            this.shooter = shooter;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;

            addRequirements(drivetrain, shooter);
        }

        @Override
        public void initialize() {
            shooter.setKickerSpeed(KICKER_SPEED_FORWARD);
        }

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            drivetrain.driveWithAzimuth(new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED));
        }

        @Override
        public void end(boolean interrupted) {
            shooter.reset();
        }
    }
}
