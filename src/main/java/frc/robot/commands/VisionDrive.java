package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionDrive extends Command {

    private final SwerveSubsystem swerve;

    private final NetworkTableEntry angleEntry;
    private final NetworkTableEntry distanceEntry;
    private double angle;
    private double distance;

    private final ProfiledPIDController omegaController = new ProfiledPIDController(1.15, 0, 0, new Constraints(4, 4));
    private final ProfiledPIDController xSpeedController = new ProfiledPIDController(1, 0, 0, new Constraints(2, 3));

    private Pose2d targetPosition;

    public VisionDrive(SwerveSubsystem swerve) {
        this.swerve = swerve;
        // Initialize the NetworkTable and entries here
        NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");

        angleEntry = visionTable.getEntry("Angle");
        distanceEntry = visionTable.getEntry("Distance");

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        findTarget();
    }

    @Override
    public void execute() {
        findTarget();

        if (distance > 0.0) {
            Pose2d currentPosition = swerve.getPose();
            double omegaSpeed = omegaController.calculate(currentPosition.getRotation().getRadians());
            double xSpeed = xSpeedController.calculate(currentPosition.getX());

            swerve.drive(new ChassisSpeeds(Math.abs(xSpeed), 0, omegaSpeed));
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop(); // Stop the robot when the command ends
    }

    private void findTarget() {
        angle = angleEntry.getDouble(0.0) * Constants.VisionConstants.cameraToDegrees;
        distance = distanceEntry.getDouble(0.0);

        if(distance > 0) {
            targetPosition = new Pose2d(
                    swerve.getPose().getX() + distance,
                    0,
                    Rotation2d.fromDegrees(angle)
            );

            omegaController.setGoal(targetPosition.getRotation().getRadians());
            xSpeedController.setGoal(targetPosition.getX());
        }
    }
}
