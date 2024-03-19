package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.Transforms.ROBOT_TO_FRONT_CAMERA;
import static frc.robot.Constants.Transforms.ROBOT_TO_REAR_CAMERA;
import static frc.robot.Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.Constants.VisionConstants.FRONT_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.REAR_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS;

public class VisionPoseEstimator {
    private final PhotonCamera frontCamera = new PhotonCamera(FRONT_CAMERA_NAME);
    private final PhotonPoseEstimator frontPoseEstimator = new PhotonPoseEstimator(
            APRIL_TAG_FIELD_LAYOUT,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            frontCamera,
            ROBOT_TO_FRONT_CAMERA
    );

    public VisionPoseEstimator() {
        frontPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        frontPoseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    }

    public Optional<EstimatedRobotPose> estimateGlobalPose(Pose2d prevEstimatedRobotPose, String camName) {
        if (FRONT_CAMERA_NAME.equals(camName))
            return estimatePose(frontPoseEstimator, frontCamera, prevEstimatedRobotPose);

        return Optional.empty();
    }

    //This uses the front camera as the rear camera DOESNT EXIST
    public Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation, Measure<Velocity<Angle>> angularVelocity) {
        if (estimation == null) {
            return VISION_MEASUREMENT_STANDARD_DEVIATIONS;
        }

        double targetFactors =
                1.d / estimation.targetsUsed.stream()
                        .mapToDouble(t -> 2.5 / (2.5 + t.getBestCameraToTarget().getTranslation().getNorm()))
                        .sum();
        double rotationFactor =
                2.d / (2.d + angularVelocity.in(RotationsPerSecond));
        double factor = 1.d / (targetFactors + rotationFactor);

        return VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(factor);
    }


    //This CAN be 1 as it will be cancelled in the confidenceCalculator method anyway. (avgDist = 0)

    private Optional<EstimatedRobotPose> estimatePose(PhotonPoseEstimator poseEstimator, PhotonCamera camera, Pose2d prevEstimatedRobotPose) {
        SmartDashboard.putBoolean("isCameraConnected", camera.isConnected());

        if (!camera.isConnected()) {
            return Optional.empty();
        }

        poseEstimator.setReferencePose(prevEstimatedRobotPose);

        return poseEstimator.update();
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
