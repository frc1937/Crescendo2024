package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionPoseEstimator;

import static frc.robot.Constants.ChaseTagPIDConstants.*;


public class ChaseTagCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final VisionPoseEstimator visionPoseEstimator;

    private final HolonomicDriveController driveController = new HolonomicDriveController(X_CONTROLLER, Y_CONTROLLER, OMEGA_CONTROLLER);


    public ChaseTagCommand(SwerveSubsystem swerveSubsystem, VisionPoseEstimator visionPoseEstimator) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionPoseEstimator = visionPoseEstimator;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d robotPose = swerveSubsystem.getPose();

        driveController.getThetaController().reset(robotPose.getRotation().getRadians());
        driveController.getXController().reset();
        driveController.getYController().reset();
    }

    @Override
    public void execute() {
        Pose2d robotPose2d = swerveSubsystem.getPose();
        Pose3d robotPose =
                new Pose3d(
                        robotPose2d.getX(),
                        robotPose2d.getY(),
                        0.0,
                        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        if (visionPoseEstimator.hasTargets()) {
            Pose2d goalPose = visionPoseEstimator.getTargetTagPose(robotPose).toPose2d();
            //.transformBy(TAG_TO_GOAL).toPose2d();

            driveController.getXController().setSetpoint(goalPose.getX());
            driveController.getYController().setSetpoint(goalPose.getY());
            driveController.getThetaController().setGoal(goalPose.getRotation().getRadians());
        }

        // Drive to the target
        double xSpeed = X_CONTROLLER.calculate(robotPose.getX());
        if (X_CONTROLLER.atSetpoint()) {
            xSpeed = 0;
        }

        double ySpeed = Y_CONTROLLER.calculate(robotPose.getY());
        if (Y_CONTROLLER.atSetpoint()) {
            ySpeed = 0;
        }

        double omegaSpeed = OMEGA_CONTROLLER.calculate(robotPose2d.getRotation().getRadians());
        if (OMEGA_CONTROLLER.atGoal()) {
            omegaSpeed = 0;
        }

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("omegaSpeed", omegaSpeed);
        //todo: CHECK LOGIC, this method below might be completely inaccurate for getting to the target pose.
        swerveSubsystem.drive(new Translation2d(X_CONTROLLER.getSetpoint(), Y_CONTROLLER.getSetpoint()), OMEGA_CONTROLLER.getGoal().position, true, false);
        // swerveSubsystem.drive(new Translation2d(-X_CONTROLLER.getGoal().position * 0.8, yController.getGoal().position * 0.8), 0.1 * robotPose2d.getRotation().getRotations(), true, true);
        //swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, 0, omegaSpeed, robotPose2d.getRotation()));
        // swerveSubsystem.drive(new ChassisSpeeds(0, ySpeed, 0));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }
}
