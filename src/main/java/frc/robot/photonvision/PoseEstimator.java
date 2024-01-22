package frc.robot.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;

public class PoseEstimator extends SubsystemBase {
//    private final PhotonCamera photonCamera;
//    public PoseEstimator(PhotonCamera photonCamera) {
//        this.photonCamera = photonCamera;
//    }
//
//    public boolean hasTarget() {
//        PhotonPipelineResult result = photonCamera.getLatestResult();
//        return result.hasTargets();
//    }
//
//    public Integer getBestTargetID() {
//        PhotonPipelineResult result = photonCamera.getLatestResult();
//
//        if(result == null || !result.hasTargets())
//            return null;
//
//        PhotonTrackedTarget target = result.getBestTarget();
//
//        if(target == null) return null;
//
//        return target.getFiducialId();
//    }

    private final PhotonCamera photonCamera;
    private final SwerveSubsystem swerveSubsystem;

     /*Ordered list of target poses by ID (WPILib is adding some functionality for
     this)*/
    private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
            new Pose3d(3.0, 1.165, 0.287 + 0.165, new Rotation3d(0, 0, degreesToRadians(180.0))),
            new Pose3d(3.0, 0.0, 0.287 + .165, new Rotation3d(0, 0, degreesToRadians(180.0)))));
/*
     Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
     you trust your various sensors. Smaller numbers will cause the filter to
     "trust" the estimate from that particular component more than the others.
     This in turn means the particualr component will have a stronger influence
     on the final pose estimate.*/

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
            * matrix is in the form [x, y, theta, s_0, ... s_n]ᵀ, with units in meters and radians, then meters.
            */
    private static final Vector<N7> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.05, 0.05, 0.05, 0.05);

    /**
     * Standard deviations of the encoder and gyro measurements. Increase these numbers to trust sensor readings from
     * encoders and gyros less. This matrix is in the form [theta, s_0, ... s_n], with units in radians followed by meters.
            */
    private static final Vector<N5> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.01), 0.01, 0.01, 0.01, 0.01);

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field2d = new Field2d();

    private double previousPipelineTimestamp = 0;

    public PoseEstimator(PhotonCamera photonCamera, SwerveSubsystem swerveSubsystem) {
        this.photonCamera = photonCamera;
        this.swerveSubsystem = swerveSubsystem;

        ShuffleboardTab tab = Shuffleboard.getTab("photonvision");

        poseEstimator = new SwerveDrivePoseEstimator(
                swerveKinematics,
                swerveSubsystem.getYaw(),
                swerveSubsystem.getModulePositions(),
                new Pose2d()
                  ,stateStdDevs,
                    visionMeasurementStdDevs
        );
                /* new SwerveDrivePoseEstimator(
                Nat.N7(),
                Nat.N7(),
                Nat.N5(),
                swerveSubsystem.getYaw(),
                swerveSubsystem.getModulePositions(),
                new Pose2d(),
                swerveKinematics,
                stateStdDevs,
                localMeasurementStdDevs,
                visionMeasurementStdDevs);
*/

        tab.addString("Pose", this::getFormattedPose).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    }

    @Override
    public void periodic() {
        // Update pose estimator with the best visible target
        PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
        double resultTimestamp = pipelineResult.getTimestampSeconds();

        if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
            previousPipelineTimestamp = resultTimestamp;
            PhotonTrackedTarget target = pipelineResult.getBestTarget();
            int fiducialID = target.getFiducialId();

            if (target.getPoseAmbiguity() <= .2 && fiducialID >= 0 && fiducialID < targetPoses.size()) {
                Pose3d targetPose = targetPoses.get(fiducialID);
                Transform3d camToTarget = target.getBestCameraToTarget();

                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
                Pose3d visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);

                poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
            }
        }

        // Update pose estimator with drivetrain sensors
        poseEstimator.update(
                swerveSubsystem.getYaw(),
                swerveSubsystem.getModulePositions());

        field2d.setRobotPose(getCurrentPose());
    }

    private String getFormattedPose() {
        Pose2d pose = getCurrentPose();
        return String.format("(%.2f, %.2f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     *
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(
                swerveSubsystem.getYaw(),
                swerveSubsystem.getModulePositions(),
                newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }
}