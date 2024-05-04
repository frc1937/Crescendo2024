package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve5990;
import frc.robot.util.AlliancePose2d;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.BLUE_SPEAKER;
import static frc.robot.Constants.RED_SPEAKER;

public class ShootOnTheMove {
    private final ShooterSubsystem shooterSubsystem;
    private final PoseEstimator5990 poseEstimator5990;
    private final ShooterCommands shooterCommands;
    private final Swerve5990 swerve5990;

    public ShootOnTheMove(ShooterSubsystem shooterSubsystem, PoseEstimator5990 poseEstimator5990, ShooterCommands shooterCommands, Swerve5990 swerve5990) {
        this.shooterSubsystem = shooterSubsystem;
        this.poseEstimator5990 = poseEstimator5990;
        this.shooterCommands = shooterCommands;
        this.swerve5990 = swerve5990;
    }


    public void shootStill(double tangentialVelocity) {
        Rotation2d theta = getThetaFromSpeeds(tangentialVelocity);

        ShooterSubsystem.Reference reference = new ShooterSubsystem.Reference(theta, MetersPerSecond.of(tangentialVelocity));

        shooterCommands.shootNote(reference);
    }


    public void shootOnTheMove(double tangentialVelocity) {
        Pose3d targetPose = AlliancePose2d.AllianceUtils.isBlueAlliance() ? BLUE_SPEAKER : RED_SPEAKER;
        Pose2d robotPose = poseEstimator5990.getCurrentPose().getCorrectPose();

        ChassisSpeeds robotVelocity = swerve5990.getSelfRelativeVelocity();
        double timeOfFlight = shooterSubsystem.getTimeOfFlight(robotPose, targetPose, tangentialVelocity);

        Translation2d targetOffset = new Translation2d(
            robotVelocity.vxMetersPerSecond * timeOfFlight,
            robotVelocity.vyMetersPerSecond * timeOfFlight
        );


    }

    private Rotation2d getThetaFromSpeeds(double tangentialVelocity) {
        Pose3d targetPose = AlliancePose2d.AllianceUtils.isBlueAlliance() ? BLUE_SPEAKER : RED_SPEAKER;
        Pose2d robotPose = poseEstimator5990.getCurrentPose().getCorrectPose();

        return shooterSubsystem.getPitchAnglePhysics(robotPose, targetPose, tangentialVelocity);
    }
}
