package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;


public class ChaseTagCommand extends Command {
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
    private final Swerve swerve;

    private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(4, 0, 0, OMEGA_CONSTRAINTS);

    public ChaseTagCommand(Swerve swerve) {
        this.swerve = swerve;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Pose2d robotPose = swerve.getPose();

        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    @Override
    public void execute() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Photon1937");

        Pose2d robotPose2d = swerve.getPose();
        Pose3d robotPose =
                new Pose3d(
                        robotPose2d.getX(),
                        robotPose2d.getY(),
                        0.0,
                        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

        if (swerve.hasTargets()) {
            Pose2d goalPose = swerve.getTagPose().toPose2d();//.transformBy(TAG_TO_GOAL).toPose2d();

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
        }//todo: get current networktable values, then try to position them by PID (they should be at -

        //These values are roughly when the tag is in the middle - need area.
        //targetPixelX - 90
        //targetPixelY - 69

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("omegaSpeed", omegaSpeed);

        //swerve.drive(new Translation2d(xController.getGoal().position, yController.getGoal().position), omegaController.getGoal().position, true, true);
        swerve.drive(new Translation2d(-xController.getGoal().position * 0.8, yController.getGoal().position * 0.8), 0.1 * robotPose2d.getRotation().getRotations(), true, true);
        //swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, 0, omegaSpeed, robotPose2d.getRotation()));
        // swerve.drive(new ChassisSpeeds(0, ySpeed, 0));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }
}
