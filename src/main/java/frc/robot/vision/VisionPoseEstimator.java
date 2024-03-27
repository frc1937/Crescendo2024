package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Camera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS;

public class VisionPoseEstimator {
    private final HashMap<PhotonPoseEstimator, PhotonCamera> poseEstimators = new HashMap<>();

    public VisionPoseEstimator(Camera... cameras) {
        PhotonPoseEstimator poseEstimator;
        PhotonCamera photonCamera;

        for (Camera camera : cameras) {
            photonCamera = new PhotonCamera(camera.name());

            poseEstimator = new PhotonPoseEstimator(
                    APRIL_TAG_FIELD_LAYOUT,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    photonCamera,
                    camera.robotToCamera()
            );

            poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
            poseEstimator.setTagModel(TargetModel.kAprilTag36h11);

            poseEstimators.put(poseEstimator, photonCamera);
        }
    }

    public List<Optional<EstimatedRobotPose>> estimateGlobalPose(Pose2d prevEstimatedRobotPose) {
        List<Optional<EstimatedRobotPose>> estimatedRobotPoses = new ArrayList<>();

        for(Map.Entry<PhotonPoseEstimator, PhotonCamera> poseEstimator : poseEstimators.entrySet()) {
            estimatedRobotPoses.add(estimatePose(poseEstimator.getKey(), poseEstimator.getValue(), prevEstimatedRobotPose));
        }

        return estimatedRobotPoses;
    }

    public Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation, Measure<Velocity<Angle>> angularVelocity) {
        if (estimation == null) {
            return VISION_MEASUREMENT_STANDARD_DEVIATIONS;
        }

        double targetFactors =
                1.d / estimation.targetsUsed.stream()
                        .mapToDouble(t -> 2.5 / (2.5 + t.getBestCameraToTarget().getTranslation().getNorm()))
                        .sum();
        double rotationFactor =
                2.d / (2.d + Math.abs(angularVelocity.in(RotationsPerSecond)));
        double factor = 1.d / (targetFactors + rotationFactor);

        return VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(factor);
    }


    private Optional<EstimatedRobotPose> estimatePose(PhotonPoseEstimator poseEstimator, PhotonCamera camera, Pose2d prevEstimatedRobotPose) {
        SmartDashboard.putBoolean("isCameraConnected " + camera.getName(), camera.isConnected());

        if (!camera.isConnected()) {
            return Optional.empty();
        }

        poseEstimator.setReferencePose(prevEstimatedRobotPose);

        return poseEstimator.update();
    }
}
