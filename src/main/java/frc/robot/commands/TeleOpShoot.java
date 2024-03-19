package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.RobotState;
import frc.lib.Target;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_FORWARD;
import static frc.robot.Constants.ShootingConstants.POST_SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.SHOOTING_PREDICTION_TIME;
import static frc.robot.Constants.Swerve.MAX_SPEED;

public class TeleOpShoot extends SequentialCommandGroup {
    public TeleOpShoot(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, Target target, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        addCommands(
                new TeleopAim(drivetrain, shooter, target, translationSup, strafeSup),
                new TeleopThrow(drivetrain, shooter, translationSup, strafeSup).withTimeout(SHOOTING_DELAY + POST_SHOOTING_DELAY)
        );
    }

    private static class TeleopAim extends Command {
        private final DrivetrainSubsystem drivetrain;
        private final ShooterSubsystem shooter;
        private final Target target;
        private final DoubleSupplier translationSup, strafeSup;

        private double virtualTargetDistance = 0;

        public TeleopAim(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, Target target, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
            this.drivetrain = drivetrain;
            this.shooter = shooter;
            this.target = target;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;

            addRequirements(drivetrain, shooter);
        }

        @Override
        public void initialize() {
            // Predict the position the robot will be in when the NOTE is released
            RobotState predictedState = drivetrain.getHistory().predict(Timer.getFPGATimestamp() + SHOOTING_DELAY);
            Translation2d targetDisplacement = target.calculateTargetDisplacement(predictedState);
            Translation2d virtualTargetDisplacement = Target.calculateVirtualTargetDisplacement(
                    targetDisplacement.getNorm(), targetDisplacement, predictedState.getVelocity());
            virtualTargetDistance = virtualTargetDisplacement.getNorm();

        }

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            // Predict the position the robot will be in when the NOTE is released
            RobotState predictedState = drivetrain.getHistory().predict(Timer.getFPGATimestamp() + SHOOTING_PREDICTION_TIME);
            Translation2d targetDisplacement = target.calculateTargetDisplacement(predictedState);

            // Calculate the displacement of the virtual target, to which the robot so it can
            // score whilst moving
            Translation2d virtualTargetDisplacement = Target.calculateVirtualTargetDisplacement(
                    virtualTargetDistance, targetDisplacement, predictedState.getVelocity());
            virtualTargetDistance = virtualTargetDisplacement.getNorm();

            SmartDashboard.putNumber("shooting/distance from target [meters]", targetDisplacement.getNorm());
            SmartDashboard.putNumber("calibration/distance from virtual target [meters]", virtualTargetDistance);

            // Aim the azimuth to the virtual target
            drivetrain.driveWithAzimuth(new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED),
                    virtualTargetDisplacement.getAngle());

            // Aim the shooter
            shooter.setReference(target.getReferenceByDistance(virtualTargetDistance));
        }

        @Override
        public boolean isFinished() {
            boolean azimuthReady = drivetrain.azimuthAtGoal();
            boolean notMoving = new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble()).getNorm() <= Constants.STICK_DEADBAND;
            boolean readyToKick = shooter.atReference() && azimuthReady;

            SmartDashboard.putBooleanArray("azimuth | not-moving", new boolean[]{azimuthReady, notMoving});

            return notMoving && readyToKick;
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
        public boolean isFinished() {
            return new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble()).getNorm() > Constants.STICK_DEADBAND;
        }

        @Override
        public void end(boolean interrupted) {
            shooter.reset();
        }
    }
}
