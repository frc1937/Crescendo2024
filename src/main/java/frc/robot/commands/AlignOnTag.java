package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionPoseEstimator;

public class AlignOnTag extends Command {
    private final VisionPoseEstimator visionPoseEstimator;
    private final SwerveSubsystem swerveSubsystem;
    private final int id;

    public AlignOnTag(VisionPoseEstimator visionPoseEstimator, SwerveSubsystem swerveSubsystem, int id) {
        this.visionPoseEstimator = visionPoseEstimator;
        this.swerveSubsystem = swerveSubsystem;
        this.id = id;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        alignOnTag(id);
    }

    public void alignOnTag(int id) {
        Pose2d robotPose2d = swerveSubsystem.getPose();
        Pose3d robotPose = new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        Pose3d target = visionPoseEstimator.getClosestTarget(robotPose, id);
        Pose2d targetPose2d = target.toPose2d();

        swerveSubsystem.pathPlanToPose(targetPose2d);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }
}
