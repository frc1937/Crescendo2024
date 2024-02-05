package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopShooting extends SequentialCommandGroup {
    public TeleopShooting(SwerveSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        addCommands(
            new TeleopAim(swerve, shooter, translationSup, strafeSup),
            new TeleopThrow(swerve, shooter, translationSup, strafeSup).withTimeout(SHOOTING_DELAY + POST_SHOOTING_DELAY)
        );
    }

    private class TeleopAim extends Command {
        private final SwerveSubsystem swerve;
        private final ShooterSubsystem shooter;
        private final DoubleSupplier translationSup, strafeSup;

        private final ProfiledPIDController yawController = /* TODO */; // TODO Get from a HononomicDriveController in other branch
        private final Rotation2d targetShooterOrientation = new Rotation2d();
        
        public TeleopAim(SwerveSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
            this.swerve = swerve;
            this.shooter = shooter;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;

            addRequirements(swerve, shooter);
        }

        @Override
        public void initialize() {
            // Start the shooter's flywheels at near-maximal speed
            shooter.startFlywheels();
        }

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

            // Predict the position the robot will be in when the NOTE is released
            RobotState predictedState = RobotState.predict(swerve.getPoseHistory(), timer.getFPGATimestamp() + SHOOTING_DELAY);

            // Use this to calculate the position the shooter will be in
            Translation3d pivotToShooter = new Translation3d(
                shooterOrientation.getCos(), 0, shooterOrientation.getSin());
            Translation3d robotToShooter = ROBOT_TO_PIVOT.plus(pivotToShooter);
            Pose3d predictedShooterPosition = new Pose3d(predictedState.getPose()).transformBy(robotToShooter);

            // Calculate the total velocity vector at which the NOTE should be thrown
            Translation3d targetNoteTranslation = TARGET_POS.minus(predictedShooterPosition);
            Translation3d targetNoteDirection = targetTranslation.div(targetTranslation.getNorm());
            Translation3d targetNoteVelocity = targetNoteDirection.times(NOTE_RELEASE_VELOCITY);

            // Factor in the robot's velocity to know how to aim the NOTE
            Translation3d throwVelocity = targetNoteVelocity.minus(predictedState.getVelocity());

            // Determine the intended release direction based on the throwVelocity, disregarding its
            // magnitude. As we consistently throw the ball at high speeds, we can simplify the path of
            // the note in space to a straight line.
            Rotation2d targetOrientation = throwVelocity.toTranslation2d.getAngle();
            double quasiTargetSlope = throwVelocity.getZ() / throwVelocity.toTranslation2d().getNorm();

            // However, becasue we do not fully trust the straight-line model, we introduce
            // a table that maps path slopes, quasiTargetSlope, to well-adjusted
            // shooter orientations. We presume the shooter orientation will be steeper than
            // the path slope becasue of gravity.
            targetShooterOrientation = SLOPE_TO_SHOOTER_ROTATION_MAP.get(quasiTargetSlope);

            swerve.drive(
                new Translation2d(translation, strafe).times(Constants.Swerve.MAX_SPEED),
                yawController.calculate(swerve.getPose().getRotation().getRadians(), throwVelocity.getAngle().getRadians()),
                true,
                true
            );
`
            // Store shooterExtension to calculate robotToShooter at the next time
            shooterExtension = throwVelocity.toTranslation2d().getNorm() / throwVelocity.getNorm();
        }

        @Override
        public boolean isFinished() {
            return shooter.flywheelsReady() && shooter.shooterAtSetpoint() && yawController.atSetpoint();
        }
    }

    private class TeleopThrow extends Command {
        private final SwerveSubsystem swerve;
        private final ShooterSubsystem shooter;
        private final DoubleSupplier translationSup, strafeSup;

        public TeleopThrow(SwerveSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
            this.swerve = swerve;
            this.shooter = shooter;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;

            addRequirements(swerve, shooter);
        }

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.g
            etAsDouble(), Constants.stickDeadband);

            swerve.drive(
                new Translation2d(targetTranslation, targetStrafe).times(Constants.Swerve.MAX_SPEED),
                0,
                true,
                true
            );
        }

        @Override
        public void end(boolean interrupted) {
            shooter.stopMotor();
        }
    }
}
