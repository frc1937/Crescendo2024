package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.poseestimation.PhotonCameraSource;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.subsystems.swerve.Swerve5990;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import static frc.robot.Constants.Transforms.ROBOT_TO_FRONT_CAMERA;

public class AlignWithTag extends SequentialCommandGroup {
    private final Swerve5990 swerve5990;
    private final PoseEstimator5990 poseEstimator5990;
    private final PhotonCameraSource photonCameraSource;

    public AlignWithTag(Swerve5990 swerve5990, PoseEstimator5990 poseEstimator5990, PhotonCameraSource photonCameraSource) {
        this.swerve5990 = swerve5990;
        this.poseEstimator5990 = poseEstimator5990;
        this.photonCameraSource = photonCameraSource;
    }

    public Command driveToTag(int id) {
        return new FunctionalCommand(
                () -> {},
                () -> swerve5990.driveFieldRelative(0, 0, swerve5990.getGyroAzimuth().plus(getTransformedTagPose(id))),
                interrupt -> {},
                () -> false,

                swerve5990
        );
    }

    private Rotation2d getTransformedTagPose(int id) {
        Pose2d robotPose = poseEstimator5990.getCurrentPose().getBluePose();
        Optional<Pose3d> tagPose = getTagPose(robotPose, id);

        if (tagPose.isEmpty()) {
            return robotPose.getRotation();
        }

        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d differenceInXY = tagPose.get().toPose2d().getTranslation().minus(robotTranslation);

        SmartDashboard.putNumber("NIG/robotTranslation", robotTranslation.getNorm());
        SmartDashboard.putNumber("NIG/tagTranslation", tagPose.get().toPose2d().getTranslation().getNorm());
        SmartDashboard.putNumber("NIG/differenceXY", differenceInXY.getNorm());

        SmartDashboard.putNumber("NIG/atan of diff", Math.atan2(Math.abs(differenceInXY.getY()), Math.abs(differenceInXY.getX())));

        SmartDashboard.putNumber("NIG/X difference: ", differenceInXY.getX());
        SmartDashboard.putNumber("NIG/Y difference; ", differenceInXY.getY());

        Rotation2d desiredAnge = Rotation2d.fromRadians(Math.atan2(
                differenceInXY.getY(),
                -differenceInXY.getX()));

        SmartDashboard.putNumber("NIG/Desired angle (From robot): ", desiredAnge.getDegrees());
        SmartDashboard.putNumber("NIG/Current angle (Of robot):", robotPose.getRotation().getDegrees());

        return desiredAnge;
    }

    private Optional<Pose3d> getTagPose(Pose2d robotPose, int id) {
        PhotonTrackedTarget target = photonCameraSource.getTags();

        if (target == null || target.getFiducialId() != id)
            return Optional.empty();

        Pose3d robotPose3d = new Pose3d(robotPose);

        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Pose3d cameraPose = robotPose3d.transformBy(ROBOT_TO_FRONT_CAMERA);

        Pose3d tagPose = cameraPose.transformBy(cameraToTarget.inverse());

        return Optional.ofNullable(tagPose);
    }
}