package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.RobotState;
import frc.lib.Targeting;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShootingConstants.BLUE_SPEAKER_TARGET;
import static frc.robot.Constants.ShootingConstants.DISTANCE_TO_REFERENCE_MAP;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_FORWARD;
import static frc.robot.Constants.ShootingConstants.POST_SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.SHOOTING_PREDICTION_TIME;
import static frc.robot.Constants.Swerve.MAX_SPEED;

public class TeleOpShoot extends SequentialCommandGroup {
    public TeleOpShoot(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, Translation2d blueTargetPosition, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        addCommands(
                new TeleopAim(drivetrain, shooter, blueTargetPosition, translationSup, strafeSup),
                new TeleopThrow(drivetrain, shooter, translationSup, strafeSup).withTimeout(SHOOTING_DELAY + POST_SHOOTING_DELAY)
        );

        SmartDashboard.putNumber("calibration/angle [deg]", 0);
        SmartDashboard.putNumber("calibration/rpm [RPM]", 0);
       SmartDashboard.putNumber("calibration/spin [idk]", 1);
    }

    private static class TeleopAim extends Command {
        private final DrivetrainSubsystem drivetrain;
        private final ShooterSubsystem shooter;
        private final DoubleSupplier translationSup, strafeSup;
        private final Translation2d blueTargetPosition;

        private final Timer deadlineTimer = new Timer();

        private double virtualTargetDistance = 0;

        public TeleopAim(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, Translation2d blueTargetPosition, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
            this.drivetrain = drivetrain;
            this.shooter = shooter;
            this.blueTargetPosition = blueTargetPosition;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;

            addRequirements(drivetrain, shooter);
        }

        @Override
        public void initialize() {
            deadlineTimer.restart();

            // Predict the position the robot will be in when the NOTE is released
            RobotState predictedState = RobotState.predict(drivetrain.getPoseHistory(), Timer.getFPGATimestamp() + SHOOTING_DELAY);
            Translation2d targetDisplacement = Targeting.calculateTargetDisplacement(predictedState, blueTargetPosition);      
            Translation2d virtualTargetDisplacement = Targeting.calculateVirtualTargetDisplacement(
                targetDisplacement.getNorm(), targetDisplacement, predictedState.getVelocity());
            virtualTargetDistance = virtualTargetDisplacement.getNorm();

        }

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            // Predict the position the robot will be in when the NOTE is released
            RobotState predictedState = RobotState.predict(drivetrain.getPoseHistory(), Timer.getFPGATimestamp() + SHOOTING_PREDICTION_TIME);
            Translation2d targetDisplacement = Targeting.calculateTargetDisplacement(predictedState, blueTargetPosition);      

            // Calculate the displacement of the virtual target, to which the robot so it can
            // score whilst moving
            Translation2d virtualTargetDisplacement = Targeting.calculateVirtualTargetDisplacement(
                virtualTargetDistance, targetDisplacement, predictedState.getVelocity());
            virtualTargetDistance = virtualTargetDisplacement.getNorm();
            
            SmartDashboard.putNumber("shooting/distance from target [meters]", targetDisplacement.getNorm());
            SmartDashboard.putNumber("calibration/distance from virtual target [meters]", virtualTargetDistance);

            // Aim the azimuth to the virtual target
            drivetrain.driveWithAzimuth(new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED),
                                    virtualTargetDisplacement.getAngle());

            // Aim the shooter
            shooter.setReference(
                   DISTANCE_TO_REFERENCE_MAP.get(virtualTargetDistance)
                    // new ShooterSubsystem.Reference(
                    //         Rotation2d.fromDegrees(SmartDashboard.getNumber("calibration/angle [deg]", 0)),
                    //         RPM.of(SmartDashboard.getNumber("calibration/rpm [RPM]", 0)),
                    //        SmartDashboard.getNumber("calibration/spin [idk]", 1)
                    // )
            );
        }

        @Override
        public boolean isFinished() {
            boolean azimuthReady = drivetrain.azimuthAtGoal();
            boolean notMoving = new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble()).getNorm() <= Constants.STICK_DEADBAND;
            boolean readyToKick = shooter.atReference() && azimuthReady;
            
            boolean reachedDeadline = deadlineTimer.hasElapsed(1 + virtualTargetDistance * 0.2);
            
            boolean flywheelsReady = shooter.flywheelsAtReference();  // TODO for debugging, remove
            boolean pitchReady = shooter.pitchAtReference();  // TODO for debugging, remove
            SmartDashboard.putBooleanArray("flywheels | pitch | azimuth | not-moving", new boolean[]{flywheelsReady, pitchReady, azimuthReady, notMoving});

            return notMoving && (readyToKick || reachedDeadline);
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
        public boolean isFinished() {
            return new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble()).getNorm() > Constants.STICK_DEADBAND;
        }

        @Override
        public void end(boolean interrupted) {
            shooter.reset();
        }
    }
}
