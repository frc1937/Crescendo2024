package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.RobotState;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShootingConstants.BLUE_TARGET_POSITION;
import static frc.robot.Constants.ShootingConstants.DEFAULT_SLOPE_TO_VIRTUAL_TARGET;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_MAX_RPM;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_FORWARD;
import static frc.robot.Constants.ShootingConstants.POST_SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.RED_TARGET_POSITION;
import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.SLOPE_TO_PITCH_MAP;
import static frc.robot.Constants.Swerve.MAX_SPEED;
import static frc.robot.Constants.Transforms.ROBOT_TO_PIVOT;
import static frc.robot.Constants.Transforms.SHOOTER_ARM_LENGTH;

public class FixedSpeedShooting extends SequentialCommandGroup {
    public FixedSpeedShooting(SwerveSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        addCommands(
                new FixedSpeedAim(swerve, shooter, translationSup, strafeSup),
                new TeleopThrow(swerve, shooter, translationSup, strafeSup).withTimeout(SHOOTING_DELAY + POST_SHOOTING_DELAY)
        );
    }

    private static class FixedSpeedAim extends Command {
        private final SwerveSubsystem swerve;
        private final ShooterSubsystem shooter;
        private final DoubleSupplier translationSup, strafeSup;
        private Rotation2d targetShooterOrientation = new Rotation2d();
        private Rotation2d orientationToVirtualTarget;
        private double slopeToVirtualTarget = DEFAULT_SLOPE_TO_VIRTUAL_TARGET;
        private final Timer deadlineTimer = new Timer();

        public FixedSpeedAim(SwerveSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
            this.swerve = swerve;
            this.shooter = shooter;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;

            addRequirements(swerve, shooter);
        }

        @Override
        public void initialize() {
            deadlineTimer.restart();
            shooter.setFlywheelSpeed(FLYWHEEL_MAX_RPM, false);
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
            Translation3d targetPosition = DriverStation.getAlliance().get() == Alliance.Red ? RED_TARGET_POSITION : BLUE_TARGET_POSITION;
            Translation3d translationToTarget = targetPosition.minus(predictedShooterPosition);
            SmartDashboard.putNumber("Distance from target [meters]", translationToTarget.toTranslation2d().getNorm());

            // Aim to the target
            slopeToVirtualTarget = translationToTarget.getZ() / translationToTarget.toTranslation2d().getNorm();
            orientationToVirtualTarget = translationToTarget.toTranslation2d().getAngle();
            SmartDashboard.putNumber("Target pitch [slope]", slopeToVirtualTarget);

            // Introduce a table that maps path slopes, slopeToVirtualTarget, to well-adjusted
            // shooter orientations. We presume the actual shooter orientation will be steeper than
            // the path slope becasue of gravity.
            targetShooterOrientation = SLOPE_TO_PITCH_MAP.get(slopeToVirtualTarget);

            swerve.driveWithAzimuth(new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED),
                                    orientationToVirtualTarget);

            shooter.setPivotAngle(targetShooterOrientation);
        }

        @Override
        public boolean isFinished() {
            return deadlineTimer.hasElapsed(1.5);
        }

        @Override
        public void end(boolean interrupted) {
           if (interrupted) {
               shooter.stopFlywheels();
               shooter.setPivotAngle(Rotation2d.fromDegrees(0));
           }

            // yawController.reset(0);
        }
    }

    public static class TeleopThrow extends Command {
        private final SwerveSubsystem swerve;
        private final ShooterSubsystem shooter;
        private final DoubleSupplier translationSup, strafeSup;

        public TeleopThrow(SwerveSubsystem swerve, ShooterSubsystem shooter,
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
            shooter.setPivotAngle(Rotation2d.fromDegrees(0));
        }
    }
}