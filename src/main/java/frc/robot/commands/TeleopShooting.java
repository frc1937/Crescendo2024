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
        private Rotation2dorientationToVirtualTarget;
        private double slopeToVirtualTarget;

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
        public void initialize() {}

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            // Predict the position the robot will be in when the NOTE is released
            RobotState predictedState = RobotState.predict(swerve.getPoseHistory(), Timer.getFPGATimestamp() + SHOOTING_DELAY);

            Translation3d predictedShooterPosition;
            if (orientationToVirtuaorientationToVirtualTarget != null) {
                predictedState.setPose(new Pose2d(predictedState.getPose().getTranslation(), orientationToVirtualTarget));

                // Use this to calculate the position the shooter will be in
                Translation3d pivotToShooter = new Translation3d(
                        targetShooterOrientation.getCos() * SHOOTER_ARM_LENGTH, 0, targetShooterOrientation.getSin() * SHOOTER_ARM_LENGTH);
                Transform3d robotToShooter = new Transform3d(ROBOT_TO_PIVOT.plus(pivotToShooter), new Rotation3d());
                predictedShooterPosition = predictedState.getPose3d().transformBy(robotToShooter).getTranslation();
            } else {
                // We cannot trust the predicted yaw. Thus, using it to predict the shooter position
                // is unviable. Using the centre of the robot is good enough. In the next time execute()
                // is called, orientationToVirtualTarget will have a value and this won't be needed.
                predictedShooterPosition = predictedState.getPose3d().getTranslation();
            }

            // Calculate the total translation vector from the shooter to the target
            Translation3d targetPosition =  DriverStation.getAlliance().get() == Alliance.Red ? RED_TARGET_POSITION : BLUE_TARGET_POSITION;
            Translation3d translationToTarget = targetPosition.minus(predictedShooterPosition);
            SmartDashboard.putNumber("Presumed distance from target [meters]", targetTranslation.toTranslation2d().getNorm());

            // Factor in the robot's velocity
            Translation3d translationDueToRobotVelocity = new Translation3d(
                    predictedState.getVelocity().vxMetersPerSecond * NOTE_TIME_IN_AIR,
                    predictedState.getVelocity().vyMetersPerSecond * NOTE_TIME_IN_AIR,
                    0
            );

            // Find the virtual target, i.e., the target to which the robot should aim s.t.
            // the NOTE enters the actual target
            Tranlation3d translationToVirtualTarget = translationToTarget.minus(translationDueToRobotVelocity);

            // Aim to the virtual target
            slopeToVirtualTarget = translationToVirtualTarget.getZ() / translationToVirtualTarget.toTranslation2d().getNorm();
            orientationToVirtualTarget = translationToVirtualTarget.toTranslation2d().getAngle();
            SmartDashboard.putNumber("orientationToVirtuaorientationToVirtualTarget", orientationToVirtualTarget.getDegrees());
            SmartDashboard.putNumber("Virtual target pitch [slope]", slopeToVirtualTarget);

            // Introduce a table that maps path slopes, slopeToVirtualTarget, to well-adjusted
            // shooter orientations. We presume the actual shooter orientation will be steeper than    
            // the path slope becasue of gravity.
            targetShooterOrientation = SLOPE_TO_PITCH_MAP.get(slopeToVirtualTarget);

            swerve.drive(
                    new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED),
                    yawController.calculate(swerve.getPose().getRotation().getRadians(), orientationToVirtualTarget.getRadians()),
                    true,
                    true
            );
            shooter.setPivotAngle(targetShooterOrientation);
            shooter.setFlywheelSpeed(SLOPE_TO_VELOCITY_MAP.get(slopeToVirtualTarget), true);
        }

        @Override
        public boolean isFinished() {
            boolean flywheelsReady = shooter.areFlywheelsReady();
            boolean pitchReady = shooter.hasPivotArrived();
            boolean yawReady = yawController.atSetpoint();
            boolean slopeViable = slopeToVirtualTarget >= MINIMUM_VIABLE_SLOPE && slopeToVirtualTarget <= MAXIMUM_VIABLE_SLOPE;

            boolean readyToKick = flywheelsReady && pitchReady && yawReady && slopeViable;

            SmartDashboard.putBooleanArray("flywheels | pitch | yaw | slope", new boolean[]{flywheelsReady, pitchReady, yawReady, slopeViable});

            return readyToKick || !shooter.doesSeeNote();
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                shooter.stopFlywheels();
                shooter.setPivotAngle(Rotation2d.fromDegrees(0));
            }

            yawController.reset(0);
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
            shooter.setKickerSpeed(0.9);
        }

        @Override
        public void execute() {
            // Get values, deadband
            double targetTranslation = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
            double targetStrafe = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);

            swerve.drive(
                    new Translation2d(targetTranslation, targetStrafe).times(MAX_SPEED),
                    0,
                    true,
                    true
            );
        }

        @Override
        public void end(boolean interrupted) {
            shooter.stopKicker();
            shooter.stopFlywheels();
            shooter.setPivotAngle(Rotation2d.fromDegrees(0));
        }
    }
}