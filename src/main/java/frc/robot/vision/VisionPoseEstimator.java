package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;

import java.util.Optional;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.Transforms.ROBOT_TO_FRONT_CAMERA;
import static frc.robot.Constants.VisionConstants.*;

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

    public Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation, Measure<Velocity<Angle>> angularVelocity) {
        if (estimation == null) {
            return VISION_MEASUREMENT_STANDARD_DEVIATIONS;
        }

        double targetFactors;

        if(estimation.targetsUsed.isEmpty()) {
            targetFactors = 0;
        } else {
            targetFactors =
                    1.d / estimation.targetsUsed.stream()
                            .mapToDouble(t -> 2.5 / (2.5 + t.getBestCameraToTarget().getTranslation().getNorm()))
                            .sum();
        }

        double rotationFactor =
                2.d / (2.d + angularVelocity.in(RotationsPerSecond));
        double factor = 1.d / (targetFactors + rotationFactor);

        return VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(factor);
    }


    private Optional<EstimatedRobotPose> estimatePose(PhotonPoseEstimator poseEstimator, PhotonCamera camera, Pose2d prevEstimatedRobotPose) {
        SmartDashboard.putBoolean("isCameraConnected", camera.isConnected());

        if (!camera.isConnected()) {
            return Optional.empty();
        }

        poseEstimator.setReferencePose(prevEstimatedRobotPose);

        return poseEstimator.update();
    }
}
