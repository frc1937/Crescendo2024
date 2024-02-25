package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import static frc.robot.Constants.Transforms.ROBOT_TO_CAMERA;
import static frc.robot.Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.Constants.VisionConstants.CAMERA_NAME;

public class VisionPoseEstimator {
    private final PhotonCamera photonCamera = new PhotonCamera(CAMERA_NAME);
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            photonCamera,
            ROBOT_TO_CAMERA);

    public VisionPoseEstimator() {
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    }

    public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (!photonCamera.isConnected()) return null;

        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();
        EstimatedRobotPose robotPose;

        Optional<EstimatedRobotPose> estimatedNonNull = estimatedRobotPose.stream().findFirst();

        if (estimatedNonNull.isPresent()) {
            robotPose = estimatedNonNull.get();
            return robotPose;
        }

        return null;
    }

    public Pose3d getClosestTarget(Pose3d robotPose, int id) {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        Pose3d tagPose = null;

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();

            if (target.getFiducialId() != id) return null;

            Transform3d cameraToTarget = target.getBestCameraToTarget();
            Pose3d cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

            tagPose = cameraPose.transformBy(cameraToTarget.inverse());
        }

        return tagPose;
    }
}
