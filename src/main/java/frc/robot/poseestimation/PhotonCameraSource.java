package frc.robot.poseestimation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.poseestimation.photonposeestimator.EstimatedRobotPose;
import frc.robot.poseestimation.photonposeestimator.PhotonPoseEstimator;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import static frc.robot.GlobalConstants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.GlobalConstants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static org.photonvision.estimation.TargetModel.kAprilTag36h11;

/**
 * A pose source is a class that provides the robot's pose, from a camera.
 */
public class PhotonCameraSource {
    private final StructArrayPublisher<Pose3d> cameraPoses = NetworkTableInstance.getDefault().getStructArrayTopic("CameraPose VISUs", Pose3d.struct).publish();

    private final PhotonCamera photonCamera;
    private final frc.robot.poseestimation.photonposeestimator.PhotonPoseEstimator photonPoseEstimator;
    private final String name;
    private final Transform3d robotToCamera;
    private double lastUpdatedTimestamp;
    private Pose2d cachedPose = null;

    private boolean hasResult = false;
    private int visibleTags = 0;
    private double averageDistanceFromTags = 0;
    private double lastResultTimestamp = 0;
    private Pose3d cameraPose = new Pose3d();

    public PhotonCameraSource(String name, Transform3d robotToCamera, PoseEstimator5990 poseEstimator5990) {
        this.name = name;
        this.robotToCamera = robotToCamera;

        photonCamera = new PhotonCamera(name);

        photonPoseEstimator = new PhotonPoseEstimator(
                APRIL_TAG_FIELD_LAYOUT,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                photonCamera,
                robotToCamera,
                poseEstimator5990
        );

        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS);
        photonPoseEstimator.setTagModel(kAprilTag36h11);
    }

    public void update() {
        if(!photonCamera.isConnected()) return;

        PhotonPipelineResult latestResult = photonCamera.getLatestResult();
        Optional<EstimatedRobotPose> optionalEstimatedRobotPose = photonPoseEstimator.update(latestResult);

        hasResult = optionalEstimatedRobotPose.filter(this::hasResult).isPresent();

        if (hasResult) {
            EstimatedRobotPose estimatedRobotPose = optionalEstimatedRobotPose.get();

            cameraPose = estimatedRobotPose.estimatedPose();
            lastResultTimestamp = estimatedRobotPose.timestampSeconds();
            visibleTags = estimatedRobotPose.targetsUsed().size();
            averageDistanceFromTags = getAverageDistanceFromTags(latestResult);
        } else {
            visibleTags = 0;
            cameraPose = new Pose3d();
        }

        cameraPoses.set(
                new Pose3d[]{
                        cameraPose

                }
        );

        SmartDashboard.putNumber("niggiiekafe", Units.radiansToDegrees(cameraPose.getRotation().getY()));


        cachedPose = cameraPose.toPose2d();


    }

    public PhotonTrackedTarget getTags() {
        if(photonCamera.getLatestResult().hasTargets())
            return photonCamera.getLatestResult().getBestTarget();

        return null;
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

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    private boolean hasResult(EstimatedRobotPose estimatedRobotPose) {
        if (estimatedRobotPose.strategy() == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
            return true;

        return estimatedRobotPose.targetsUsed().get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD;
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
