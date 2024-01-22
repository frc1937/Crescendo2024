package frc.robot.photonvision;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;

public class ChaseTag extends CommandBase {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private static final int TAG_TO_CHASE = 2;
    private static final Transform3d TAG_TO_GOAL =
            new Transform3d(
                    new Translation3d(1.5, 0.0, 0.0),
                    new Rotation3d(0.0, 0.0, Math.PI));

    private final PhotonCamera photonCamera;
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Pose2d> poseProvider;

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;

    public ChaseTag(
            PhotonCamera photonCamera,
            SwerveSubsystem swerveSubsystem,
            Supplier<Pose2d> poseProvider) {
        this.photonCamera = photonCamera;
        this.swerveSubsystem = swerveSubsystem;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        Pose2d robotPose = poseProvider.get();

        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = poseProvider.get();
        Pose3d robotPose =
                new Pose3d(
                        robotPose2d.getX(),
                        robotPose2d.getY(),
                        0.0,
                        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        PhotonPipelineResult photonRes = photonCamera.getLatestResult();

        if (photonRes.hasTargets()) {
            // Find the tag we want to chase

            Optional<PhotonTrackedTarget> targetOpt = photonRes.getTargets().stream()
                    .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
                    .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
                    .findFirst();

            if (targetOpt.isPresent()) {
                PhotonTrackedTarget target = targetOpt.get();
                // This is new target data, so recalculate the goal
                lastTarget = target;

                // Transform the robot's pose to find the camera's pose
                Pose3d cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

                // Trasnform the camera's pose to the target's pose
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d targetPose = cameraPose.transformBy(camToTarget);

                // Transform the tag's pose to set our goal
                Pose2d goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

                // Drive
                xController.setGoal(goalPose.getX());
                yController.setGoal(goalPose.getY());
                omegaController.setGoal(goalPose.getRotation().getRadians());
            }
        }

        if (lastTarget == null) {
            // No target has been visible
            swerveSubsystem.stop(); //stop it
        } else {
            // Drive to the target
            double xSpeed = xController.calculate(robotPose.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }

            double ySpeed = yController.calculate(robotPose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }

            double omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }

            Translation2d translation2d = new Translation2d(xSpeed, ySpeed);
            //todo: Test this, might be incorrect.
            swerveSubsystem.drive(translation2d, omegaSpeed, true, true);
            //  ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }
}
