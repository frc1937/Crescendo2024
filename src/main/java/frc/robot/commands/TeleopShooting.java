package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import static frc.robot.Constants.ShootingConstants.MAXIMUM_VIABLE_SLOPE;
import static frc.robot.Constants.ShootingConstants.MINIMUM_VIABLE_SLOPE;
import static frc.robot.Constants.ShootingConstants.NOTE_RELEASE_VELOCITY;
import static frc.robot.Constants.ShootingConstants.POST_SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.RED_TARGET_POSITION;
import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.SLOPE_TO_PITCH_MAP;
import static frc.robot.Constants.ShootingConstants.SLOPE_TO_VELOCITY_MAP;
import static frc.robot.Constants.Swerve.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.Swerve.MAX_SPEED;
import static frc.robot.Constants.Swerve.YAW_CONTROLLER_D;
import static frc.robot.Constants.Swerve.YAW_CONTROLLER_I;
import static frc.robot.Constants.Swerve.YAW_CONTROLLER_P;
import static frc.robot.Constants.Transforms.ROBOT_TO_PIVOT;
import static frc.robot.Constants.Transforms.SHOOTER_ARM_LENGTH;

public class TeleopShooting extends SequentialCommandGroup {
    public TeleopShooting(SwerveSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        addCommands(
                new TeleopAim(swerve, shooter, translationSup, strafeSup),
                new TeleopThrow(swerve, shooter, translationSup, strafeSup).withTimeout(SHOOTING_DELAY + POST_SHOOTING_DELAY)
        );
    }

    private static class TeleopAim extends Command {
        private final SwerveSubsystem swerve;
        private final ShooterSubsystem shooter;
        private final DoubleSupplier translationSup, strafeSup;
        private final ProfiledPIDController yawController = new ProfiledPIDController(YAW_CONTROLLER_P, YAW_CONTROLLER_I, YAW_CONTROLLER_D,
                new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ANGULAR_VELOCITY));  // WARNING this is nuts.
        private Rotation2d targetShooterOrientation = new Rotation2d();
        private Rotation2d targetOrientation;
        private double virtualTargetSlope;

        public TeleopAim(SwerveSubsystem swerve, ShooterSubsystem shooter, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
            this.swerve = swerve;
            this.shooter = shooter;
            this.translationSup = translationSup;
            this.strafeSup = strafeSup;

            yawController.setTolerance(0.1); // TODO move to constants
            yawController.enableContinuousInput(-Math.PI, Math.PI);

            addRequirements(swerve, shooter);
        }

        @Override
        public void initialize() {
            // Start the shooter's flywheels at maximal speed
            shooter.setFlywheelSpeed(1.d);
        }

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            // Predict the position the robot will be in when the NOTE is released
            RobotState predictedState = RobotState.predict(swerve.getPoseHistory(), Timer.getFPGATimestamp() + SHOOTING_DELAY);

            Translation3d predictedShooterPosition;
            if (targetOrientation != null) {
                predictedState.setPose(new Pose2d(predictedState.getPose().getTranslation(), targetOrientation));

                // Use this to calculate the position the shooter will be in
                Translation3d pivotToShooter = new Translation3d(
                        targetShooterOrientation.getCos() * SHOOTER_ARM_LENGTH, 0, targetShooterOrientation.getSin() * SHOOTER_ARM_LENGTH);
                Transform3d robotToShooter = new Transform3d(ROBOT_TO_PIVOT.plus(pivotToShooter), new Rotation3d());
                predictedShooterPosition = predictedState.getPose3d().transformBy(robotToShooter).getTranslation();
            } else {
                // We cannot trust the predicted yaw. Thus, using it to predict the shooter position
                // is unviable. Using the centre of the robot is good enough. In the next time execute()
                // is called, targetOrientation will have a value and this won't be needed.
                predictedShooterPosition = predictedState.getPose3d().getTranslation();
            }
            // RobotState predictedState = new RobotState(swerve.getPose(),
            //         new ChassisSpeeds(targetTranslation * 1, targetStrafe * 1
            //         , 0));


            // Calculate the total velocity vector at which the NOTE should be thrown
            Translation3d targetPosition =  DriverStation.getAlliance().get() == Alliance.Red ? RED_TARGET_POSITION : BLUE_TARGET_POSITION;
            Translation3d targetNoteTranslation = targetPosition.minus(predictedShooterPosition);
            SmartDashboard.putNumber("Presumed distance from target [meters]", targetNoteTranslation.toTranslation2d().getNorm());
            Translation3d targetNoteDirection = targetNoteTranslation.div(targetNoteTranslation.getNorm());
            Translation3d targetNoteVelocity = targetNoteDirection.times(NOTE_RELEASE_VELOCITY);

            // Factor in the robot's velocity to know how to aim the NOTE
            Translation3d predictedVelocity = new Translation3d(
                    predictedState.getVelocity().vxMetersPerSecond,
                    predictedState.getVelocity().vyMetersPerSecond,
                    0
            );
            // Translation2d predictedVel = predictedState.getPose().getTranslation().minus(swerve.getPose().getTranslation()).div(SHOOTING_DELAY);
            // Translation3d predictedVelocity = new Translation3d(targetTranslation, targetStrafe, 0);
            SmartDashboard.putNumberArray("predictedVelocity", new double[]{predictedVelocity.getX(), predictedVelocity.getY()});
            Translation3d throwVelocity = targetNoteVelocity.minus(predictedVelocity);

            // Determine the intended release direction based on the throwVelocity, disregarding its
            // magnitude. As we consistently throw the ball at high speeds, we can simplify the path of
            // the note in space to a straight line.
            targetOrientation = throwVelocity.toTranslation2d().getAngle();
            SmartDashboard.putNumber("targetOrientation", targetOrientation.getDegrees());
            virtualTargetSlope = throwVelocity.getZ() / throwVelocity.toTranslation2d().getNorm();
            SmartDashboard.putNumber("Virtual target pitch [slope]", virtualTargetSlope);

            // However, becasue we do not fully trust the straight-line model, we introduce
            // a table that maps path slopes, virtualTargetSlope, to well-adjusted
            // shooter orientations. We presume the shooter orientation will be steeper than
            // the path slope becasue of gravity.
            targetShooterOrientation = SLOPE_TO_PITCH_MAP.get(virtualTargetSlope);

            swerve.drive(
                    new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED),
                    yawController.calculate(swerve.getPose().getRotation().getRadians(), targetOrientation.getRadians()),
                    true
            );
            shooter.setPivotAngle(targetShooterOrientation);
        }

        @Override
        public boolean isFinished() {
            boolean flywheelsReady = shooter.areFlywheelsReady(SLOPE_TO_VELOCITY_MAP.get(virtualTargetSlope));
            boolean pitchReady = shooter.hasPivotArrived();
            boolean shooterNotOccluded = !shooter.isOccluded();
            boolean yawReady = yawController.atSetpoint();
            boolean slopeViable = virtualTargetSlope >= MINIMUM_VIABLE_SLOPE && virtualTargetSlope <= MAXIMUM_VIABLE_SLOPE;

            SmartDashboard.putBooleanArray("flywheels | pitch | not occluded | yaw | slope", new boolean[]{flywheelsReady, pitchReady, shooterNotOccluded, yawReady, slopeViable});

            return flywheelsReady && pitchReady && shooterNotOccluded && yawReady && slopeViable;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                shooter.stopFlywheels();
            }

            yawController.reset(0);
            //targetShooterOrientation = new Rotation2d();
            //targetOrientation = new Rotation2d();
        }
    }

    private static class TeleopThrow extends Command {
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
        public void initialize() {
            shooter.setKickerSpeed(0.7);
        }

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            swerve.drive(
                    new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED),
                    0,
                    true
            );
        }

        @Override
        public void end(boolean interrupted) {
            shooter.stopKicker();
            shooter.stopFlywheels();
        }
    }
}