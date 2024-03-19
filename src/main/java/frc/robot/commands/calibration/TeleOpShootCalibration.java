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
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.POST_SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.SHOOTING_PREDICTION_TIME;
import static frc.robot.Constants.Swerve.MAX_SPEED;

public class TeleOpShootCalibration extends SequentialCommandGroup {
    public TeleOpShootCalibration(DrivetrainSubsystem drivetrain, ShooterSubsystem shooter, Target target, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        addCommands(
                new TeleopAim(drivetrain, shooter, target, translationSup, strafeSup),
                new TeleOpShoot.TeleopThrow(drivetrain, shooter, translationSup, strafeSup).withTimeout(SHOOTING_DELAY + POST_SHOOTING_DELAY)
        );

        SmartDashboard.putNumber("calibration/angle [deg]", 0);
        SmartDashboard.putNumber("calibration/rpm [RPM]", 0);
        SmartDashboard.putNumber("calibration/spin [idk]", 1);
    }

    private static class TeleopAim extends Command {
        private final DrivetrainSubsystem drivetrain;
        private final ShooterSubsystem shooter;
        private final Target target;
        private final DoubleSupplier translationSup, strafeSup;

        private final Timer deadlineTimer = new Timer();

        private double targetDistance = 0;

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
            deadlineTimer.restart();
        }

        @Override
        public void execute() {
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            RobotState predictedState = drivetrain.getHistory().predict(Timer.getFPGATimestamp() + SHOOTING_PREDICTION_TIME);
            Translation2d targetDisplacement = target.calculateTargetDisplacement(predictedState);

            SmartDashboard.putNumber("calibration/distance from target [meters]", targetDisplacement.getNorm());

            drivetrain.driveWithAzimuth(new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED),
                    targetDisplacement.getAngle());

            // Aim the shooter
            shooter.setReference(
                    new ShooterSubsystem.Reference(
                            Rotation2d.fromDegrees(SmartDashboard.getNumber("calibration/angle [deg]", 0)),
                            RPM.of(SmartDashboard.getNumber("calibration/rpm [RPM]", 0)),
                            SmartDashboard.getNumber("calibration/spin [idk]", 1)
                    ));
        }

        @Override
        public boolean isFinished() {
            boolean azimuthReady = drivetrain.azimuthAtGoal();
            boolean notMoving = new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble()).getNorm() <= Constants.STICK_DEADBAND;
            boolean readyToKick = shooter.atReference() && azimuthReady;

            boolean reachedDeadline = deadlineTimer.hasElapsed(1 + targetDistance * 0.2);

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
}
