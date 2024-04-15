package frc.robot.poseestimation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.AllianceUtilities;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import static frc.robot.Constants.VisionConstants.*;

/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator implements AutoCloseable {
    private final Field2d field = new Field2d();
    private final PhotonCameraSource[] photonCameraSources;
    private final PoseEstimator6328 swerveDrivePoseEstimator;
    private final DrivetrainSubsystem drivetrainSubsystem;
    private AllianceUtilities.AlliancePose2d robotPose = DEFAULT_POSE;

    /**
     * Constructs a new PoseEstimator.
     *
     * @param photonCameraSources the sources that should update the pose estimator apart from the odometry. This should be cameras etc.
     */
    public PoseEstimator(PoseEstimator6328 swerveDrivePoseEstimator, DrivetrainSubsystem drivetrainSubsystem, PhotonCameraSource... photonCameraSources) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.photonCameraSources = photonCameraSources;
        this.swerveDrivePoseEstimator = swerveDrivePoseEstimator;

        putAprilTagsOnFieldWidget();
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void close() {
        field.close();
    }

    public void periodic() {
        updateFromVision();
        robotPose = AllianceUtilities.AlliancePose2d.fromBlueAlliancePose(swerveDrivePoseEstimator.getEstimatedPose());
        field.setRobotPose(getCurrentPose().toBlueAlliancePose());
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to, as an {@link AllianceUtilities.AlliancePose2d}
     */
    public void resetPose(AllianceUtilities.AlliancePose2d currentPose) {
        final Pose2d currentBluePose = currentPose.toBlueAlliancePose();
        drivetrainSubsystem.setHeading(currentBluePose.getRotation());
        swerveDrivePoseEstimator.resetPose(currentBluePose);
    }

    /**
     * @return the estimated pose of the robot, as an {@link AllianceUtilities.AlliancePose2d}
     */
    public AllianceUtilities.AlliancePose2d getCurrentPose() {
        return robotPose;
    }

    /**
     * Updates the pose estimator with the given swerve wheel positions and gyro rotations.
     * This function accepts an array of swerve wheel positions and an array of gyro rotations because the odometry can be updated at a faster rate than the main loop (which is 50 hertz).
     * This means you could have a couple of odometry updates per main loop, and you would want to update the pose estimator with all of them.
     *
     * @param swerveWheelPositions the swerve wheel positions accumulated since the last update
     * @param gyroRotations        the gyro rotations accumulated since the last update
     */
    public void updateFromOdometry(SwerveDriveWheelPositions[] swerveWheelPositions, Rotation2d[] gyroRotations, double[] timestamps) {
        for (int i = 0; i < swerveWheelPositions.length; i++)
            swerveDrivePoseEstimator.addOdometryObservation(new PoseEstimator6328.OdometryObservation(swerveWheelPositions[i], gyroRotations[i], timestamps[i]));
    }

    private void updateFromVision() {
        getViableVisionObservations().stream()
                .sorted(Comparator.comparingDouble(PoseEstimator6328.VisionObservation::timestamp))
                .forEach(swerveDrivePoseEstimator::addVisionObservation);
    }

    private List<PoseEstimator6328.VisionObservation> getViableVisionObservations() {
        List<PoseEstimator6328.VisionObservation> viableVisionObservations = new ArrayList<>();

        for (PhotonCameraSource photonCameraSource : photonCameraSources) {
            final PoseEstimator6328.VisionObservation visionObservation = getVisionObservation(photonCameraSource);
            if (visionObservation != null)
                viableVisionObservations.add(visionObservation);
        }

        return viableVisionObservations;
    }

    private PoseEstimator6328.VisionObservation getVisionObservation(PhotonCameraSource photonCameraSource) {
        photonCameraSource.update();

        if (!photonCameraSource.hasNewResult())
            return null;

        Pose2d robotPoseFromSource = photonCameraSource.getRobotPose();
        if (robotPoseFromSource == null)
            return null;

        return new PoseEstimator6328.VisionObservation(
                robotPoseFromSource,
                photonCameraSource.getLastResultTimestamp(),
                averageDistanceToStdDevs(photonCameraSource.getAverageDistanceFromTags(), photonCameraSource.getVisibleTags())
        );
    }

    private Matrix<N3, N1> averageDistanceToStdDevs(double averageDistance, int visibleTags) {
        final double translationStd = TRANSLATION_STD_EXPONENT * Math.pow(averageDistance, 2) / (visibleTags * visibleTags);
        final double thetaStd = ROTATION_STD_EXPONENT * Math.pow(averageDistance, 2) / visibleTags;

        return VecBuilder.fill(translationStd, translationStd, thetaStd);
    }

    private void putAprilTagsOnFieldWidget() {
        for (Map.Entry<Integer, Pose3d> entry : TAG_ID_TO_POSE.entrySet()) {
            field.getObject("Tag " + entry.getKey()).setPose(entry.getValue().toPose2d());
        }
    }
}