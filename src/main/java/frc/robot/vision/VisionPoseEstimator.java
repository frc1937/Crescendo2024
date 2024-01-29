package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.Optional;

import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;

public class VisionPoseEstimator {
    private final PhotonCamera photonCamera = new PhotonCamera("Photon1937");
    private final AprilTagFieldLayout aprilTagFieldLayout = getAprilTagFieldLayout();
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE
            ,photonCamera,
            ROBOT_TO_CAMERA);

    public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
        EstimatedRobotPose robotPose;

        if (estimatedRobotPose.stream().findFirst().isPresent()) {
            robotPose = estimatedRobotPose.stream().findFirst().get();
            return robotPose;
        }

        return null;
    }

    public Pose3d getTargetTagPose(Pose3d robotPose) {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        Pose3d tagPose = new Pose3d();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Pose3d cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

            tagPose = cameraPose.transformBy(cameraToTarget.inverse());

            //todo: Remove debugging below.
            SmartDashboard.putNumber("Target X: ", tagPose.getX());
            SmartDashboard.putNumber("Target Y: ", tagPose.getY());

            SmartDashboard.putNumber("CURR X: ", robotPose.getX());
            SmartDashboard.putNumber("CURR Y: ", robotPose.getY());
        }

        return tagPose;
    }

    public boolean hasTargets() {
        return photonCamera.getLatestResult().hasTargets();
    }

    private AprilTagFieldLayout getAprilTagFieldLayout() {
        AprilTagFieldLayout fieldLayout;
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        return fieldLayout;
    }
}
