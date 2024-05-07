package frc.robot.poseestimation;

import edu.wpi.first.math.geometry.*;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import static frc.robot.Constants.VisionConstants.*;

/**
 * A pose source is a class that provides the robot's pose, from a camera.
 */
public class PhotonCameraSource {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private final String name;
    private final Transform3d robotToCamera;
    private double lastUpdatedTimestamp;
    private Pose2d cachedPose = null;

    private boolean hasResult = false;
    private int visibleTags = 0;
    private double averageDistanceFromTags = 0;
    private double lastResultTimestamp = 0;
    private Pose3d cameraPose = new Pose3d();

    public PhotonCameraSource(String name, Transform3d robotToCamera) {
        this.name = name;
        this.robotToCamera = robotToCamera;

        photonCamera = new PhotonCamera(name);

        photonPoseEstimator = new PhotonPoseEstimator(
                APRIL_TAG_FIELD_LAYOUT,
                PRIMARY_POSE_STRATEGY,
                photonCamera,
                robotToCamera
        );

        photonPoseEstimator.setMultiTagFallbackStrategy(SECONDARY_POSE_STRATEGY);
        photonPoseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    }

    public void update() {
        if(!photonCamera.isConnected()) return;

        PhotonPipelineResult latestResult = photonCamera.getLatestResult();
        Optional<EstimatedRobotPose> optionalEstimatedRobotPose = photonPoseEstimator.update(latestResult);

        hasResult = optionalEstimatedRobotPose.filter(this::hasResult).isPresent();

        if (hasResult) {
            EstimatedRobotPose estimatedRobotPose = optionalEstimatedRobotPose.get();

            cameraPose = estimatedRobotPose.estimatedPose;
            lastResultTimestamp = estimatedRobotPose.timestampSeconds;
            visibleTags = estimatedRobotPose.targetsUsed.size();
            averageDistanceFromTags = getAverageDistanceFromTags(latestResult);
        } else {
            visibleTags = 0;
            cameraPose = new Pose3d();
        }

        cachedPose = getUnCachedRobotPose();
    }

    public String getName() {
        return name;
    }

    public int getVisibleTags() {
        return visibleTags;
    }

    public double getAverageDistanceFromTags() {
        return averageDistanceFromTags;
    }

    public boolean hasNewResult() {
        return (hasResult && averageDistanceFromTags != 0) && isNewTimestamp();
    }

    public Pose2d getRobotPose() {
        return cachedPose;
    }

    public double getLastResultTimestamp() {
        return lastResultTimestamp;
    }

    private Pose2d getUnCachedRobotPose() {
        if (cameraPose == null)
            return null;

        return cameraPose.transformBy(robotToCamera.inverse()).toPose2d();
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    private boolean hasResult(EstimatedRobotPose estimatedRobotPose) {
        if (estimatedRobotPose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
            return true;

        return estimatedRobotPose.targetsUsed.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD;
    }

    private double getAverageDistanceFromTags(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.targets;
        double distanceSum = 0;

        for (PhotonTrackedTarget currentTarget : targets) {
            final Translation2d distanceTranslation = currentTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
            distanceSum += distanceTranslation.getNorm();
        }

        return distanceSum / targets.size();
    }
}
