//package frc.robot.commands;
//
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.geometry.Translation3d;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.lib.RobotState;
//import frc.robot.Constants;
//import frc.robot.subsystems.ShooterSubsystem;
//import frc.robot.subsystems.SwerveSubsystem;
//
//import java.util.function.DoubleSupplier;
//
//import static frc.robot.Constants.ShootingConstants.NOTE_RELEASE_VELOCITY;
//import static frc.robot.Constants.ShootingConstants.POST_SHOOTING_DELAY;
//import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
//import static frc.robot.Constants.ShootingConstants.SLOPE_TO_SHOOTER_ROTATION_MAP;
//import static frc.robot.Constants.ShootingConstants.TARGET_POSITION;
//import static frc.robot.Constants.Transforms.ROBOT_TO_PIVOT;
//
//public class TeleopShooting extends SequentialCommandGroup {
//    public TeleopShooting(SwerveSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
//        addCommands(
//                new TeleopAim(swerve, shooter, translationSup, strafeSup),
//                new TeleopThrow(swerve, shooter, translationSup, strafeSup).withTimeout(SHOOTING_DELAY + POST_SHOOTING_DELAY)
//        );
//    }
//
//    private static class TeleopAim extends Command {
//        private final SwerveSubsystem swerve;
//        private final ShooterSubsystem shooter;
//        private final DoubleSupplier translationSup, strafeSup;
////todo: Will this exist tho..
//        private final ProfiledPIDController yawController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2, 2));
//        /* TODO */; // TODO Get from a HononomicDriveController in other branch
//        private Rotation2d shooterOrientation = new Rotation2d();
//
//        public TeleopAim(SwerveSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
//            this.swerve = swerve;
//            this.shooter = shooter;
//            this.translationSup = translationSup;
//            this.strafeSup = strafeSup;
//
//            addRequirements(swerve, shooter);
//        }
//
//        @Override
//        public void initialize() {
//            // Start the shooter's flywheels at near-maximal speed
//            shooter.startFlywheels();
//        }
//
//        @Override
//        public void execute() {
//            // Get values, deadband
//            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
//            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
//
//            // Predict the position the robot will be in when the NOTE is released
//            RobotState predictedState = RobotState.predict(swerve.getPoseHistory(), Timer.getFPGATimestamp() + SHOOTING_DELAY);
//
//            // Use this to calculate the position the shooter will be in
//            Transform3d pivotToShooter = new Transform3d(
//                    shooterOrientation.getCos(), 0, shooterOrientation.getSin(), new Rotation3d());
//            Transform3d robotToShooter = ROBOT_TO_PIVOT.plus(pivotToShooter);
//            Pose3d predictedShooterPosition = new Pose3d(predictedState.getPose()).transformBy(robotToShooter);
//
//            // Calculate the total velocity vector at which the NOTE should be thrown
//            Translation3d targetNoteTranslation = TARGET_POSITION.minus(predictedShooterPosition.getTranslation());
//            Translation3d targetNoteDirection = targetNoteTranslation.div(targetNoteTranslation.getNorm());
//            Translation3d targetNoteVelocity = targetNoteDirection.times(NOTE_RELEASE_VELOCITY);
//
//            // Factor in the robot's velocity to know how to aim the NOTE
//            //TODO: This won't work. Velocity in what direction?
//            Translation3d throwVelocity = targetNoteVelocity.minus(predictedState.getVelocity());
//
//            // Determine the intended release direction based on the throwVelocity, disregarding its
//            // magnitude. As we consistently throw the ball at high speeds, we can simplify the path of
//            // the note in space to a straight line.
//            Rotation2d targetOrientation = throwVelocity.toTranslation2d().getAngle();
//            double quasiTargetSlope = throwVelocity.getZ() / throwVelocity.toTranslation2d().getNorm();
//
//            // However, becasue we do not fully trust the straight-line model, we introduce
//            // a table that maps path slopes, quasiTargetSlope, to well-adjusted
//            // shooter orientations. We presume the shooter orientation will be steeper than
//            // the path slope becasue of gravity.
//            shooterOrientation = SLOPE_TO_SHOOTER_ROTATION_MAP.get(quasiTargetSlope);
//
//            swerve.drive(
//                    new Translation2d(targetTranslation, targetStrafe).times(Constants.Swerve.MAX_SPEED),
//                    yawController.calculate(swerve.getPose().getRotation().getRadians(), targetOrientation.getRadians()),
//                    true
//            );
//        }
//
//        @Override
//        public boolean isFinished() {
//            return shooter.areFlywheelsReady() && shooter.hasPivotArrived() && yawController.atSetpoint();
//        }
//    }
//
//    private static class TeleopThrow extends Command {
//        private final SwerveSubsystem swerve;
//        private final ShooterSubsystem shooter;
//        private final DoubleSupplier translationSup, strafeSup;
//
//        public TeleopThrow(SwerveSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
//            this.swerve = swerve;
//            this.shooter = shooter;
//            this.translationSup = translationSup;
//            this.strafeSup = strafeSup;
//
//            addRequirements(swerve, shooter);
//        }
//
//        @Override
//        public void execute() {
//            // Get values, deadband
//            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
//            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
//
//            swerve.drive(
//                    new Translation2d(targetTranslation, targetStrafe).times(Constants.Swerve.MAX_SPEED),
//                    0,
//                    true
//            );
//        }
//
//        @Override
//        public void end(boolean interrupted) {
//            shooter.stopFlywheels();
//        }
//    }
//}