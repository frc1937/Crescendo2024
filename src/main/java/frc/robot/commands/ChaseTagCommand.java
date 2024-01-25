package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;


public class ChaseTagCommand extends Command {
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
    private final PhotonCamera photonCamera;
    private final Swerve swerve;

    private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(3, 0, 0, OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;

    public ChaseTagCommand(PhotonCamera photonCamera, Swerve swerve) {
        this.photonCamera = photonCamera;
        this.swerve = swerve;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        lastTarget = null;
        Pose2d robotPose = swerve.getPose();

        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = swerve.getPose();
        Pose3d robotPose =
                new Pose3d(
                        robotPose2d.getX(),
                        robotPose2d.getY(),
                        0.0,
                        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        PhotonPipelineResult photonRes = photonCamera.getLatestResult();
        if (photonRes.hasTargets()) {
//            Optional<PhotonTrackedTarget> targetOpt = photonRes.getTargets().stream()
//                    .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
//                    .findFirst();

            PhotonTrackedTarget target = photonRes.getBestTarget();

            lastTarget = target;

            Pose3d cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

            Transform3d camToTarget = target.getBestCameraToTarget();
            Pose3d targetPose = cameraPose.transformBy(camToTarget);

            Pose2d goalPose = targetPose.toPose2d();

            System.out.println("ITER------------------");
            System.out.println("TargetPose: " + goalPose);
            System.out.println("CurrentPose: " + robotPose2d);
            System.out.println("TION------------------");


            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());
        }

        if (lastTarget == null) {
            swerve.stop();
        } else {
            SmartDashboard.putNumber("tagDriving ", lastTarget.getFiducialId());
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
//   todo :  Choose which way is the correct to drive the robot

//            double translationValue = MathUtil.applyDeadband(, Constants.stickDeadband);
//            double strafeValue = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
//            double rotationValue = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

            //       swerve.drive(new Translation2d(xController.getGoal().position, yController.getGoal().position), omegaController.getGoal().position, true, true);

            swerve.drive(new Translation2d(xController.getGoal().position, yController.getGoal().position), omegaSpeed, true, true);
            //    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
