package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.RobotState;
import frc.lib.Target;
import frc.robot.Constants;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve5990;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShootingConstants.*;

public class TeleOpShoot extends ParallelDeadlineGroup {
    public TeleOpShoot(Swerve5990 swerve5990, ShooterSubsystem shooter, LEDsSubsystem leds, Target target, DoubleSupplier translationSup, DoubleSupplier strafeSup, boolean shootingWhilstMoving, Measure<Time> deadline) {
        super(
            new SequentialCommandGroup(
                new TeleopAim(swerve5990, shooter, target, translationSup, strafeSup, shootingWhilstMoving, deadline),
                new TeleopThrow(swerve5990, shooter, translationSup, strafeSup, shootingWhilstMoving).withTimeout(SHOOTING_DELAY + POST_SHOOTING_DELAY)
            )//,
            //new Pointing(leds, shooter)
        );
    }

    private static class TeleopAim extends Command {
        private final Swerve5990 swerve5990;
        private final ShooterSubsystem shooter;
        private final Target target;
        private final DoubleSupplier translationSup,
                                     strafeSup;
        private final boolean shootingWhilstMoving;
        private final Measure<Time> deadline;

        private final Timer deadlineTimer = new Timer();

        private double virtualTargetDistance = 0;

        public TeleopAim(Swerve5990 swerve5990, ShooterSubsystem shooter, Target target, DoubleSupplier translationSup, DoubleSupplier strafeSup, boolean shootingWhilstMoving, Measure<Time> deadline) {
            this.swerve5990 = swerve5990;
            this.shooter = shooter;
            this.target = target;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;
            this.shootingWhilstMoving = shootingWhilstMoving;
            this.deadline = deadline;

            addRequirements(swerve5990, shooter);
        }

        @Override
        public void initialize() {
            deadlineTimer.restart();

            // Predict the position the robot will be in when the NOTE is released
            RobotState predictedState = swerve5990.getHistory().predict(Timer.getFPGATimestamp() + SHOOTING_DELAY);
            Translation2d targetDisplacement = target.calculateTargetDisplacement(predictedState);
            Translation2d virtualTargetDisplacement = target.calculateVirtualTargetDisplacement(
                    targetDisplacement.getNorm(), targetDisplacement, predictedState.getVelocity());
            virtualTargetDistance = virtualTargetDisplacement.getNorm();

            swerve5990.setupAzimuthController();
        }

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            // Predict the position the robot will be in when the NOTE is released
            // RobotState predictedState = drivetrain.getHistory().predict(Timer.getFPGATimestamp() + SHOOTING_PREDICTION_TIME);
            RobotState predictedState = swerve5990.getHistory().estimate();
            Translation2d targetDisplacement = target.calculateTargetDisplacement(predictedState);

            // Calculate the displacement of the virtual target, to which the robot so it can
            // score whilst moving
            Translation2d virtualTargetDisplacement = target.calculateVirtualTargetDisplacement(
                    virtualTargetDistance, targetDisplacement, predictedState.getVelocity());
            virtualTargetDistance = virtualTargetDisplacement.getNorm();

            SmartDashboard.putNumber("calibration/distance from target [meters]", targetDisplacement.getNorm());
            SmartDashboard.putNumber("calibration/distance from virtual target [meters]", virtualTargetDistance);
            SmartDashboard.putNumber("calibration/filtered distance from virtual target [meters]", virtualTargetDistance);

            // Aim the azimuth to the virtual target
            swerve5990.driveSelfRelative(targetTranslation, targetStrafe,
                    virtualTargetDisplacement.getAngle());

            // Aim the shooter
            shooter.setReference(target.getReferenceByDistance(virtualTargetDistance));
        }

        @Override
        public boolean isFinished() {
            boolean azimuthReady = swerve5990.azimuthAtGoal(target.getAzimuthTolerance(virtualTargetDistance));
            boolean notMoving = new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble()).getNorm() <= Constants.STICK_DEADBAND;
            boolean readyToKick = shooter.atReference() && azimuthReady;

            boolean reachedDeadline = deadlineTimer.hasElapsed(deadline.in(Seconds));

            SmartDashboard.putBooleanArray("azimuth | not-moving | pitch | flywheels", new boolean[]{azimuthReady, notMoving, shooter.pitchAtReference(), shooter.flywheelsAtReference()});

            return (shootingWhilstMoving || notMoving) && (readyToKick || reachedDeadline);
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                shooter.reset();
            }
        }
    }

    public static class TeleopThrow extends Command {
        private final ShooterSubsystem shooter;
        private final Swerve5990 swerve5990;
        private final DoubleSupplier translationSup, strafeSup;
        private final boolean shootingWhilstMoving;

        public TeleopThrow(Swerve5990 swerve5990, ShooterSubsystem shooter,
                           DoubleSupplier translationSup, DoubleSupplier strafeSup,
                           boolean shootingWhilstMoving) {
            this.shooter = shooter;
            this.swerve5990 = swerve5990;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;
            this.shootingWhilstMoving = shootingWhilstMoving;

            addRequirements(swerve5990, shooter);
        }

        @Override
        public void execute() {
            if (shootingWhilstMoving) {
                // Get values, deadband
                double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
                double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

                swerve5990.driveFieldRelative(targetTranslation, targetStrafe, 0);
            }
        }

        @Override
        public void initialize() {
            shooter.setKickerSpeed(KICKER_SPEED_FORWARD);
        }

        @Override
        public boolean isFinished() {
            if (shootingWhilstMoving) {
                return false;
            } else {
                return new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble()).getNorm() > Constants.STICK_DEADBAND;
            }
        }

        @Override
        public void end(boolean interrupted) {
            shooter.reset();
        }
    }
}
