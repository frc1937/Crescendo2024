package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionPoseEstimator;

import static frc.robot.Constants.ChaseTagPIDConstants.OMEGA_CONTROLLER;
import static frc.robot.Constants.ChaseTagPIDConstants.X_CONTROLLER;
import static frc.robot.Constants.ChaseTagPIDConstants.Y_CONTROLLER;


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
            //todo: find out more about desiredLinearVelocity
            ChassisSpeeds speedOutput = driveController.calculate(robotPose2d, goalPose, 2, goalPose.getRotation());

            swerveSubsystem.drive(speedOutput);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }
}
