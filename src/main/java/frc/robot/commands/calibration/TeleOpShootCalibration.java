package frc.robot.commands.calibration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.RobotState;
import frc.lib.Target;
import frc.robot.Constants;
import frc.robot.commands.TeleOpShoot;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve5990;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class TeleOpShootCalibration extends SequentialCommandGroup {
    public TeleOpShootCalibration(Swerve5990 swerve5990, ShooterSubsystem shooter, Target target, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        addCommands(
                new TeleopAim(swerve5990, shooter, target, translationSup, strafeSup),
                new TeleOpShoot.TeleopThrow(swerve5990, shooter, translationSup, strafeSup, false).withTimeout(SHOOTING_DELAY + POST_SHOOTING_DELAY)
        );

        SmartDashboard.putNumber("calibration/angle [deg]", 0);
        SmartDashboard.putNumber("calibration/rpm [RPM]", 0);
        SmartDashboard.putNumber("calibration/spin [idk]", 1);
    }

    private static class TeleopAim extends Command {
        private final Swerve5990 swerve5990;
        private final ShooterSubsystem shooter;
        private final Target target;
        private final DoubleSupplier translationSup, strafeSup;

        private final Timer deadlineTimer = new Timer();

        private double targetDistance = 0;

        public TeleopAim(Swerve5990 swerve5990, ShooterSubsystem shooter, Target target, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
            this.swerve5990 = swerve5990;
            this.shooter = shooter;
            this.target = target;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;

            addRequirements(swerve5990, shooter);
        }

        @Override
        public void initialize() {
            deadlineTimer.restart();
        }

        @Override
        public void execute() {
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            RobotState predictedState = swerve5990.getStateHistory().predict(Timer.getFPGATimestamp() + SHOOTING_PREDICTION_TIME);
            Translation2d targetDisplacement = target.calculateTargetDisplacement(predictedState);

            SmartDashboard.putNumber("calibration/distance from target [meters]", targetDisplacement.getNorm());

            swerve5990.driveFieldRelative(targetTranslation, targetStrafe,
                    targetDisplacement.getAngle());

            // Aim the shooter
            shooter.setReference(
                    new ShooterSubsystem.Reference(
                            Rotation2d.fromDegrees(SmartDashboard.getNumber("calibration/angle [deg]", 0)),
                            MetersPerSecond.of(SmartDashboard.getNumber("calibration/mps [MetresPerSecond]", 0))
                    ));
        }

        @Override
        public boolean isFinished() {
            boolean azimuthReady = swerve5990.azimuthAtGoal(target.getAzimuthTolerance(targetDistance));
            boolean notMoving = new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble()).getNorm() <= Constants.STICK_DEADBAND;
            boolean readyToKick = shooter.atReference() && azimuthReady;

            boolean reachedDeadline = deadlineTimer.hasElapsed(1 + targetDistance * 0.2);

            boolean flywheelsReady = shooter.flywheelsAtReference();  // TODO for debugging, remove
            boolean pitchReady = shooter.pitchAtReference();  // TODO for debugging, remove
            SmartDashboard.putBooleanArray("flywheels | pitch | azimuth | not-moving", new boolean[]{flywheelsReady, pitchReady, notMoving});

            return notMoving && (readyToKick || reachedDeadline);
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                shooter.reset();
            }
        }
    }
}
