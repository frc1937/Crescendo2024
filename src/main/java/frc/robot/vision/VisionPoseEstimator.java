package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
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

    public Optional<EstimatedRobotPose> estimateGlobalPose(Pose2d prevEstimatedRobotPose, String camName) {
        if (FRONT_CAMERA_NAME.equals(camName))
            return estimatePose(frontPoseEstimator, frontCamera, prevEstimatedRobotPose);
        if (REAR_CAMERA_NAME.equals(camName))
            return estimatePose(rearPoseEstimator, rearCamera, prevEstimatedRobotPose);

        return Optional.empty();
    }

    //This uses the front camera as the rear camera DOESNT EXIST
    public Matrix<N3, N1> confidenceCalculator() {
        final double averageDistance = getAverageDistanceFromTags(frontCamera.getLatestResult());
        final int visibleTags = getVisibleTags(frontCamera.getLatestResult());

        final double translationStd = 0.01 * Math.pow(averageDistance, 2) / visibleTags;
        final double thetaStd = 0.01 * Math.pow(averageDistance, 2) / visibleTags;

        return VecBuilder.fill(translationStd, translationStd, thetaStd);
    }

    //This CAN be 1 as it will be cancelled in the confidenceCalculator method anyway. (avgDist = 0)
    private int getVisibleTags(PhotonPipelineResult result) {
        return result.targets.isEmpty() ? 1 : result.targets.size();
    }

    private double getAverageDistanceFromTags(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.targets;
        double distanceSum = 0;
        Translation2d distanceTranslation;

        for (PhotonTrackedTarget currentTarget : targets) {
            distanceTranslation = currentTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
            distanceSum += distanceTranslation.getNorm();
        }

        return distanceSum / targets.size();
    }

    private Optional<EstimatedRobotPose> estimatePose(PhotonPoseEstimator poseEstimator, PhotonCamera camera, Pose2d prevEstimatedRobotPose) {
        SmartDashboard.putBoolean("isCameraConnected", camera.isConnected());

        if (!camera.isConnected()) return Optional.empty();

        poseEstimator.setReferencePose(prevEstimatedRobotPose);

        return poseEstimator.update();
    }

    private Pose3d getClosestTarget(Pose3d robotPose, int id) {
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

//    private static double getConfidenceMultiplier(EstimatedRobotPose robotPose, Measure<Distance> smallestDistance) {
//        double poseAmbiguityFactor = robotPose.targetsUsed.size() != 1
//                ? 1
//                : Math.max(
//                1,
//                (robotPose.targetsUsed.get(0).getPoseAmbiguity()
//                        + Constants.VisionConstants.POSE_AMBIGUITY_SHIFTER)
//                        * Constants.VisionConstants.POSE_AMBIGUITY_MULTIPLIER);
//        return Math.max(
//                1,
//                (Math.max(
//                        1,
//                        Math.max(0, smallestDistance.in(Meters) - Constants.VisionConstants.NOISY_DISTANCE_METERS) * Constants.VisionConstants.DISTANCE_WEIGHT)
//                        * poseAmbiguityFactor)
//                        / (1
//                        + ((robotPose.targetsUsed.size() - 1) * Constants.VisionConstants.TAG_PRESENCE_WEIGHT)));
//    }

//    public Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
//        if (estimation == null) return VISION_MEASUREMENT_STANDARD_DEVIATIONS;
//
//        double closestTargetDistanceMeters =
//                estimation.targetsUsed.stream()
//                        .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
//                        .min()
//                        .orElse(0);
//
//        Measure<Distance> closestTargetDistance = Meters.of(closestTargetDistanceMeters);
//
//        double confidenceMultiplier = getConfidenceMultiplier(estimation, closestTargetDistance);
//        return Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
//    }
}
