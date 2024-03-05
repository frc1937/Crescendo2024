package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import static frc.robot.Constants.Transforms.ROBOT_TO_FRONT_CAMERA;
import static frc.robot.Constants.Transforms.ROBOT_TO_REAR_CAMERA;
import static frc.robot.Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.Constants.VisionConstants.FRONT_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.REAR_CAMERA_NAME;

public class VisionPoseEstimator {
    private final PhotonCamera frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
    private final PhotonCamera rearCamera = new PhotonCamera(REAR_CAMERA_NAME);
    private final PhotonPoseEstimator frontPoseEstimator = new PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            frontCamera,
            ROBOT_TO_FRONT_CAMERA
    );
    private final PhotonPoseEstimator rearPoseEstimator = new PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            rearCamera,
            ROBOT_TO_REAR_CAMERA
    );

    public VisionPoseEstimator() {
        frontPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        frontPoseEstimator.setTagModel(TargetModel.kAprilTag36h11);

        rearPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        rearPoseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    }

    public EstimatedRobotPose estimateGlobalPoseFrontCam(Pose2d prevEstimatedRobotPose) {
        return estimatePose(frontPoseEstimator, frontCamera, prevEstimatedRobotPose);
    }

    public EstimatedRobotPose estimateGlobalPoseRearCam(Pose2d prevEstimatedRobotPose) {
        return estimatePose(rearPoseEstimator, rearCamera, prevEstimatedRobotPose);
    }

    private EstimatedRobotPose estimatePose(PhotonPoseEstimator poseEstimator, PhotonCamera camera, Pose2d prevEstimatedRobotPose) {
        SmartDashboard.putBoolean("isCameraConnected", camera.isConnected());

        if (!camera.isConnected()) return null;

        poseEstimator.setReferencePose(prevEstimatedRobotPose);

        Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update();
        EstimatedRobotPose robotPose;

        Optional<EstimatedRobotPose> estimatedNonNull = estimatedRobotPose.stream().findFirst();

        if (estimatedNonNull.isPresent()) {
            robotPose = estimatedNonNull.get();
            return robotPose;
        }

        return null;
    }

    public Pose3d getClosestTarget(Pose3d robotPose, int id) {
        PhotonPipelineResult result = frontCamera.getLatestResult();
        Pose3d tagPose = null;

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();

            if (target.getFiducialId() != id) return null;

            Transform3d cameraToTarget = target.getBestCameraToTarget();
            Pose3d cameraPose = robotPose.transformBy(ROBOT_TO_FRONT_CAMERA);

            tagPose = cameraPose.transformBy(cameraToTarget.inverse());
        }

        return tagPose;
    }
}
