package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve5990;
import frc.robot.util.AlliancePose2d;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.BLUE_SPEAKER;
import static frc.robot.Constants.RED_SPEAKER;

public class ShootOnTheMove extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final PoseEstimator5990 poseEstimator5990;
    private final ShooterCommands shooterCommands;
    private final Swerve5990 swerve5990;

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
    }

    @Override
    public void execute() {
        Pose3d targetPose = AlliancePose2d.AllianceUtils.isBlueAlliance() ? BLUE_SPEAKER : RED_SPEAKER;
        Pose2d robotPose = poseEstimator5990.getCurrentPose().getCorrectPose();

        ChassisSpeeds robotVelocity = swerve5990.getSelfRelativeVelocity();
        double timeOfFlight = shooterSubsystem.getTimeOfFlight(robotPose, targetPose, tangentialVelocity);

        Transform3d targetOffset = new Transform3d(
                robotVelocity.vxMetersPerSecond * timeOfFlight,
                robotVelocity.vyMetersPerSecond * timeOfFlight,
                0,
                new Rotation3d()
        );

        Pose3d newTarget = targetPose.transformBy(targetOffset.inverse());

        //Target angle for the robot to face pose
        Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(Math.abs(newTarget.getY() - robotPose.getY()), Math.abs(newTarget.getX() - robotPose.getX())));

        //Todo here:
        swerve5990.driveWithTargetAzimuth(translationSupplier.getAsDouble(), strafeSupplier.getAsDouble(), targetAngle);
        shootNote(robotPose, newTarget, tangentialVelocity);
    }

    @Override
    public boolean isFinished() {
        return swerve5990.azimuthAtGoal(Radians.of(1)) && shooterSubsystem.atReference(); //TODO: Move tolerance to constants
    }

    private void shootNote(Pose2d robotPose, Pose3d targetPose, double tangentialVelocity) {
        Rotation2d theta = shooterSubsystem.getPitchAnglePhysics(robotPose, targetPose, tangentialVelocity);
        ShooterSubsystem.Reference reference = new ShooterSubsystem.Reference(theta, MetersPerSecond.of(tangentialVelocity));

        shooterCommands.shootNote(reference);
    }
}
