package frc.robot.poseestimation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve5990;
import frc.robot.util.AllianceUtilities;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.Constants.VisionConstants.DEFAULT_POSE;
import static frc.robot.Constants.VisionConstants.ROTATION_STD_EXPONENT;
import static frc.robot.Constants.VisionConstants.TAG_ID_TO_POSE;
import static frc.robot.Constants.VisionConstants.TRANSLATION_STD_EXPONENT;

/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator5990 implements AutoCloseable {
    private final Field2d field = new Field2d();
    private final PhotonCameraSource[] photonCameraSources;
    private final PoseEstimator6328 swerveDrivePoseEstimator;
    private final Swerve5990 drivetrainSubsystem;
    private final Lock updateLock = new ReentrantLock();
    private final Notifier updateFromOdometryNotifier;
    private AllianceUtilities.AlliancePose2d robotPose = DEFAULT_POSE;

    /**
     * Constructs a new PoseEstimator.
     *
     * @param photonCameraSources the sources that should update the pose estimator apart from the odometry. This should be cameras etc.
     */
    public PoseEstimator5990(PoseEstimator6328 swerveDrivePoseEstimator, Swerve5990 drivetrainSubsystem, PhotonCameraSource... photonCameraSources) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.photonCameraSources = photonCameraSources;
        this.swerveDrivePoseEstimator = swerveDrivePoseEstimator;

        putAprilTagsOnFieldWidget();
        SmartDashboard.putData("Field", field);

        updateFromOdometryNotifier = new Notifier(this::updateFromOdometry);
        updateFromOdometryNotifier.startPeriodic(1 / 250.0);
    }

    @Override
    public void close() {
        field.close();
        updateFromOdometryNotifier.close();
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

    private void updateFromOdometry() {
        updateLock.lock();

        final SwerveDriveWheelPositions swerveDriveWheelPositions = drivetrainSubsystem.getModulePositions();
        final Rotation2d gyroYaw = drivetrainSubsystem.getYaw();
        swerveDrivePoseEstimator.addOdometryObservation(new PoseEstimator6328.OdometryObservation(
                swerveDriveWheelPositions,
                gyroYaw,
                Timer.getFPGATimestamp()
        ));

        updateLock.unlock();
    }

    private void updateFromVision() {
        updateLock.lock();

        getViableVisionObservations().stream()
                .sorted(Comparator.comparingDouble(PoseEstimator6328.VisionObservation::timestamp))
                .forEach(swerveDrivePoseEstimator::addVisionObservation);

        updateLock.unlock();
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