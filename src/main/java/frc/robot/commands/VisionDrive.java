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
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;

public class VisionDrive extends Command {

    private final SwerveSubsystem SwerveSubsystem;

    private NetworkTable visionTable;
    private NetworkTableEntry angleEntry;
    private NetworkTableEntry distanceEntry;
    private double angle;
    private double distance;

    private ProfiledPIDController omegaController = new ProfiledPIDController(1.15, 0, 0, new Constraints(4, 4));
    private ProfiledPIDController xSpeedController = new ProfiledPIDController(2.5, 0, 0, new Constraints(2, 3));

    private Pose2d currentPosition;
    private Pose2d targetPosition;

    public VisionDrive(SwerveSubsystem SwerveSubsystem) {
        this.SwerveSubsystem = SwerveSubsystem;
        // Initialize the NetworkTable and entries here
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        angleEntry = visionTable.getEntry("Angle");
        distanceEntry = visionTable.getEntry("Distance");
        addRequirements(SwerveSubsystem);
    }

    @Override
    public void initialize() {
        angle = angleEntry.getDouble(0.0) * Constants.VisionConstants.cameraToDegrees; // 170 / 360 = 0.47222 = camera_degrees / 360
        distance = distanceEntry.getDouble(0.0);
    
        if (distance != 0.0){
            targetPosition = new Pose2d(
                SwerveSubsystem.getPose().getX() + distance, 
                0, Rotation2d.fromDegrees(angle));
    
            omegaController.setGoal(targetPosition.getRotation().getRadians());
            xSpeedController.setGoal(targetPosition.getX());
        }
    }
    
    @Override
    public void execute() {
        angle = angleEntry.getDouble(0.0) * Constants.VisionConstants.cameraToDegrees; // 170 / 360 = 0.47222 = camera_degrees / 360
        distance = distanceEntry.getDouble(0.0);
        if (distance != 0.0) {
            targetPosition = new Pose2d(
                SwerveSubsystem.getPose().getX() + distance, 
                0, Rotation2d.fromDegrees(angle));
            omegaController.setGoal(targetPosition.getRotation().getRadians());
            xSpeedController.setGoal(targetPosition.getX());

            currentPosition = SwerveSubsystem.getPose();
            double omegaSpeed = omegaController.calculate(currentPosition.getRotation().getRadians());
            double xSpeed = xSpeedController.calculate(currentPosition.getX());

            if (omegaController.atGoal())
                omegaSpeed = 0;
            if (xSpeedController.atGoal())
                xSpeed = 0;

            SwerveSubsystem.drive(new ChassisSpeeds(xSpeed, 0, omegaSpeed));
        }
    }

    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.stop(); // Stop the robot when the command ends
    }

}