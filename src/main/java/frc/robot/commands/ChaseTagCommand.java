package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionPoseEstimator;


public class ChaseTagCommand extends Command {
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
    private final SwerveSubsystem swerveSubsystem;
    private final VisionPoseEstimator visionPoseEstimator;

    private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(4, 0, 0, OMEGA_CONSTRAINTS);

    public ChaseTagCommand(SwerveSubsystem swerveSubsystem, VisionPoseEstimator visionPoseEstimator) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionPoseEstimator = visionPoseEstimator;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d robotPose = swerveSubsystem.getPose();

        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
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

            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());
        }

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

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("omegaSpeed", omegaSpeed);

        //swerveSubsystem.drive(new Translation2d(xController.getGoal().position, yController.getGoal().position), omegaController.getGoal().position, true, true);
        swerveSubsystem.drive(new Translation2d(-xController.getGoal().position * 0.8, yController.getGoal().position * 0.8), 0.1 * robotPose2d.getRotation().getRotations(), true, true);
        //swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, 0, omegaSpeed, robotPose2d.getRotation()));
        // swerveSubsystem.drive(new ChassisSpeeds(0, ySpeed, 0));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }
}
