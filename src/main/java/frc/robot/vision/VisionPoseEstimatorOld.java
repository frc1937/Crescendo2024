//package frc.robot.vision;

//TODO: This class is left for future reference.

//import edu.wpi.first.apriltag.AprilTagFieldLayout;
//import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot;
//import frc.robot.subsystems.SwerveSubsystem;
//import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonPoseEstimator;
//import org.photonvision.targeting.PhotonPipelineResult;
//import org.photonvision.targeting.PhotonTrackedTarget;
//
//import java.io.IOException;
//import java.util.Optional;
//
//import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;
//import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;
//
//public class VisionPoseEstimator {
//    private final SwerveSubsystem swerve;
//    private final AprilTagFieldLayout aprilTagFieldLayout;
//    private final PhotonPoseEstimator photonPoseEstimator;
//    private final PhotonCamera photonCamera = new PhotonCamera("Photon1937");
//    private Pose3d aprilTagPosition = new Pose3d();
//    private double previousTimestamp = 0;
//
//    private Pose3d lastPose;
//    private Pose3d referencePose;
//
//    public VisionPoseEstimator(SwerveSubsystem swerveSubsystem) {
//        try {
//            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
//
//        this.swerve = swerveSubsystem;
//        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, ROBOT_TO_CAMERA);
//    }
//
//    /**
//     * Ran periodically by the {@link Robot}
//     */
//    public void periodic() {
//        PhotonPipelineResult result = photonCamera.getLatestResult();
//        double currentTimestamp = result.getTimestampSeconds();
//
//        if (result.hasTargets()) {
//            PhotonTrackedTarget target = result.getBestTarget();
//            Transform3d cameraToTarget = target.getBestCameraToTarget();
//
//            Pose3d cameraPose = swerve.getPose3d().transformBy(ROBOT_TO_CAMERA);
//
//            aprilTagPosition = cameraPose.transformBy(cameraToTarget.inverse());
//
//            SmartDashboard.putNumber("Target X: ", aprilTagPosition.getX());
//            SmartDashboard.putNumber("Target Y: ", aprilTagPosition.getY());
//
//            SmartDashboard.putNumber("CURR X: ", swerve.getPose().getX());
//            SmartDashboard.putNumber("CURR Y: ", swerve.getPose().getY());
//        }
//
//        if (currentTimestamp != previousTimestamp && result.hasTargets()) {
//            previousTimestamp = currentTimestamp;
//
//            PhotonTrackedTarget target = result.getBestTarget();
//            int fiducialId = target.getFiducialId();
//
//            Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
//
//            if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
//                Pose3d targetPose = tagPose.get();
//                Transform3d cameraToTarget = target.getBestCameraToTarget();
//                Pose3d cameraPose = targetPose.transformBy(cameraToTarget.inverse());
//
//                Pose3d visionMeasurement = cameraPose.transformBy(CAMERA_TO_ROBOT);
//                // poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), currentTimestamp);
//            }
//        }
//    }
//
//    public Pose3d getReferencePose() {
//        return referencePose;
//    }
//
//    public void setReferencePose(Pose3d referencePose) {
//        referencePose = this.referencePose;
//    }
//    public void setLastPose(Pose3d lastPose) {
//        this.lastPose = lastPose;
//    }
//
//
//    public Pose3d getTagPose() {
//        return aprilTagPosition;
//    }
//
//    public boolean hasTargets() {
//        return photonCamera.getLatestResult().hasTargets();
//    }
//
//}
