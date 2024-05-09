package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.subsystems.shooter.ShooterPhysicsCalculations;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve5990;
import frc.robot.util.AlliancePose2d;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.VisionConstants.BLUE_SPEAKER;
import static frc.robot.Constants.VisionConstants.RED_SPEAKER;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_AZIMUTH_TOLERANCE;

public class ShootOnTheMove extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final PoseEstimator5990 poseEstimator5990;
    private final ShooterCommands shooterCommands;
    private final Swerve5990 swerve5990;
    private final ShooterPhysicsCalculations shooterPhysicsCalculations;

    private final DoubleSupplier translationSupplier, strafeSupplier;
    private final double tangentialVelocity;

    public ShootOnTheMove(ShooterSubsystem shooterSubsystem, PoseEstimator5990 poseEstimator5990, ShooterCommands shooterCommands, Swerve5990 swerve5990, DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier, double tangentialVelocity) {
        this.shooterSubsystem = shooterSubsystem;
        this.poseEstimator5990 = poseEstimator5990;
        this.shooterCommands = shooterCommands;
        this.swerve5990 = swerve5990;

        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.tangentialVelocity = tangentialVelocity;

        shooterPhysicsCalculations = new ShooterPhysicsCalculations(shooterSubsystem);
    }

    @Override
    public void execute() {
        double targetTranslation = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.STICK_DEADBAND);
        double targetStrafe = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.STICK_DEADBAND);

        Pose3d targetPose = AlliancePose2d.AllianceUtils.isBlueAlliance() ? BLUE_SPEAKER : RED_SPEAKER;
        Pose2d robotPose = poseEstimator5990.getCurrentPose().getCorrectPose();

        Pose3d newTarget = shooterPhysicsCalculations.getNewTargetFromRobotVelocity(robotPose, targetPose, tangentialVelocity, swerve5990.getSelfRelativeVelocity());

        Rotation2d targetAngle = shooterPhysicsCalculations.getAzimuthAngleToTarget(robotPose, newTarget);

        swerve5990.driveWithTargetAzimuth(targetTranslation, targetStrafe, targetAngle);
        shootNote(robotPose, newTarget, tangentialVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooterSubsystem.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return swerve5990.azimuthAtGoal(Radians.of(SWERVE_AZIMUTH_TOLERANCE)) && shooterSubsystem.atReference();
    }

    private void shootNote(Pose2d robotPose, Pose3d targetPose, double tangentialVelocity) {
        Rotation2d theta = shooterPhysicsCalculations.getPitchAnglePhysics(robotPose, targetPose, tangentialVelocity);
        ShooterSubsystem.Reference reference = new ShooterSubsystem.Reference(theta, MetersPerSecond.of(tangentialVelocity));

        shooterCommands.shootNote(reference);
    }
}
