package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.RobotState;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.*;
import static frc.robot.Constants.Swerve.MAX_SPEED;
import static frc.robot.Constants.Transforms.ROBOT_TO_PIVOT;
import static frc.robot.Constants.Transforms.SHOOTER_ARM_LENGTH;

public class TeleopShooting extends SequentialCommandGroup {
    public TeleopShooting(DrivetrainSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        addCommands(
                new TeleopAim(swerve, shooter, translationSup, strafeSup),
                new TeleopThrow(swerve, shooter, translationSup, strafeSup).withTimeout(SHOOTING_DELAY + POST_SHOOTING_DELAY)
        );

        SmartDashboard.putNumber("swerve/velocity [rpm]", 3000);
        SmartDashboard.putNumber("swerve/orientation [deg]", 70);
    }

    private static class TeleopAim extends Command {
        private final DrivetrainSubsystem swerve;
        private final ShooterSubsystem shooter;
        private final DoubleSupplier translationSup, strafeSup;
        private Rotation2d targetShooterOrientation = new Rotation2d();
        private Rotation2d orientationToVirtualTarget;
        private double slopeToVirtualTarget = DEFAULT_SLOPE_TO_VIRTUAL_TARGET;
        private final Timer deadlineTimer = new Timer();

        public TeleopAim(DrivetrainSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
            this.swerve = swerve;
            this.shooter = shooter;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;

            addRequirements(swerve, shooter);
        }

        @Override
        public void initialize() {
            deadlineTimer.restart();
        }

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            // Predict the position the robot will be in when the NOTE is released
            RobotState predictedState = RobotState.predict(swerve.getPoseHistory(), Timer.getFPGATimestamp() + SHOOTING_DELAY);

            Translation3d predictedShooterPosition;
            if (orientationToVirtualTarget != null) {
                predictedState.setPose(new Pose2d(predictedState.getPose().getTranslation(), orientationToVirtualTarget));

                // Use this to calculate the position the shooter will be in
                Translation3d pivotToShooter = new Translation3d(
                        targetShooterOrientation.getCos() * SHOOTER_ARM_LENGTH, 0, targetShooterOrientation.getSin() * SHOOTER_ARM_LENGTH);
                Transform3d robotToShooter = new Transform3d(ROBOT_TO_PIVOT.plus(pivotToShooter), new Rotation3d());
                predictedShooterPosition = predictedState.getPose3d().transformBy(robotToShooter).getTranslation();
            } else {
                // We cannot trust the predicted azimuth. Thus, using it to predict the shooter position
                // is unviable. Using the centre of the robot is good enough. In the next time execute()
                // is called, orientationToVirtualTarget will have a value and this won't be needed.
                predictedShooterPosition = predictedState.getPose3d().getTranslation();
            }

            // Calculate the total translation vector from the shooter to the target
            Translation3d targetPosition = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red ? RED_TARGET_POSITION : BLUE_TARGET_POSITION;
            Translation3d translationToTarget = targetPosition.minus(predictedShooterPosition);
            SmartDashboard.putNumber("Presumed distance from target [meters]", translationToTarget.toTranslation2d().getNorm());

            // Factor in the robot's velocity and the NOTE's time of flight
            double timeOfFlight = SLOPE_TO_TIME_OF_FLIGHT_MAP.get(slopeToVirtualTarget);
            Translation3d translationDueToRobotVelocity = new Translation3d(
                    predictedState.getVelocity().vxMetersPerSecond * timeOfFlight,
                    predictedState.getVelocity().vyMetersPerSecond * timeOfFlight,
                    0
            );

            // Find the virtual target, i.e., the target to which the robot should aim s.t.
            // the NOTE enters the actual target
            Translation3d translationToVirtualTarget = translationToTarget.minus(translationDueToRobotVelocity);

            // Aim to the virtual target
            slopeToVirtualTarget = translationToVirtualTarget.getZ() / translationToVirtualTarget.toTranslation2d().getNorm();
            orientationToVirtualTarget = translationToVirtualTarget.toTranslation2d().getAngle();
//            SmartDashboard.putNumber("orientationToVirtuaorientationToVirtualTarget", orientationToVirtualTarget.getDegrees());
            SmartDashboard.putNumber("swerve/Virtual target pitch [slope]", slopeToVirtualTarget);

            // Introduce a table that maps path slopes, slopeToVirtualTarget, to well-adjusted
            // shooter orientations. We presume the actual shooter orientation will be steeper than
            // the path slope becasue of gravity.
            targetShooterOrientation =
//                    Rotation2d.fromDegrees(SmartDashboard.getNumber("swerve/orientation [deg]", 0));
                    SLOPE_TO_PITCH_MAP.get(slopeToVirtualTarget);

            swerve.driveWithAzimuth(new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED),
                                    orientationToVirtualTarget);

            shooter.setPitchPosition(targetShooterOrientation);
            shooter.setFlywheelsSpeed(RPM.of(
//                    SmartDashboard.getNumber("swerve/velocity [rpm]", 0))
                    SLOPE_TO_VELOCITY_MAP.get(slopeToVirtualTarget))
                    , SHOOTING_SPIN);
        }

        @Override
        public boolean isFinished() {
            boolean flywheelsReady = shooter.areFlywheelsReady();
            boolean pitchReady = shooter.isPitchReady();
            boolean azimuthReady = swerve.azimuthAtGoal();
            boolean slopeViable = slopeToVirtualTarget >= MINIMUM_VIABLE_SLOPE && slopeToVirtualTarget <= MAXIMUM_VIABLE_SLOPE;
            boolean reachedDeadline = deadlineTimer.hasElapsed(2.5);

            boolean readyToKick = flywheelsReady && pitchReady && azimuthReady && slopeViable;

            SmartDashboard.putBooleanArray("flywheels | pitch | azimuth | slope", new boolean[]{flywheelsReady, pitchReady, azimuthReady, slopeViable});

            return readyToKick || reachedDeadline;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                shooter.stopFlywheels();
                shooter.setPitchPosition(Rotation2d.fromDegrees(0));
            }
        }
    }

    public static class TeleopThrow extends Command {
        private final DrivetrainSubsystem swerve;
        private final ShooterSubsystem shooter;
        private final DoubleSupplier translationSup, strafeSup;

        public TeleopThrow(DrivetrainSubsystem swerve, ShooterSubsystem shooter,
                           DoubleSupplier translationSup, DoubleSupplier strafeSup) {
            this.swerve = swerve;
            this.shooter = shooter;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;

            addRequirements(swerve, shooter);
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

            swerve.driveWithAzimuth(new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED));
        }

        @Override
        public void end(boolean interrupted) {
            shooter.stopKicker();
            shooter.stopFlywheels();
            shooter.setPitchPosition(Rotation2d.fromDegrees(0));
        }
    }
}