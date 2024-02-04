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
import frc.robot.subsystems.Swerve;

public class VisionDrive extends Command {

    private final Swerve swerve;

    private NetworkTable visionTable;
    private NetworkTableEntry angleEntry;
    private NetworkTableEntry distanceEntry;
    private double angle;
    private double distance;

    private ProfiledPIDController omegaController = new ProfiledPIDController(1.15, 0, 0, new Constraints(4, 4));
    private ProfiledPIDController xSpeedController = new ProfiledPIDController(2.5, 0, 0, new Constraints(2, 3));

    private Pose2d currentPosition;
    private Pose2d targetPosition;

    public VisionDrive(Swerve swerve) {
        this.swerve = swerve;
        // Initialize the NetworkTable and entries here
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        angleEntry = visionTable.getEntry("Angle");
        distanceEntry = visionTable.getEntry("Distance");
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        angle = angleEntry.getDouble(0.0) * 0.273;
        distance = distanceEntry.getDouble(0.0);
    
        if (distance != 0.0){
            targetPosition = new Pose2d(
                swerve.getPose().getX() + distance, 
                0, Rotation2d.fromDegrees(angle));
    
            omegaController.setGoal(targetPosition.getRotation().getRadians());
            xSpeedController.setGoal(targetPosition.getX());
        }
    }
    
    @Override
    public void execute() {
        angle = angleEntry.getDouble(0.0) * 0.273; // 70 / 256 = 0.273 = camera_degrees / custom_unit
        distance = distanceEntry.getDouble(0.0);
        if (distance != 0.0) {
            targetPosition = new Pose2d(
                swerve.getPose().getX() + distance, 
                0, Rotation2d.fromDegrees(angle));
            omegaController.setGoal(targetPosition.getRotation().getRadians());
            xSpeedController.setGoal(targetPosition.getX());

            currentPosition = swerve.getPose();
            double omegaSpeed = omegaController.calculate(currentPosition.getRotation().getRadians());
            double xSpeed = xSpeedController.calculate(currentPosition.getX());

            if (omegaController.atGoal())
                omegaSpeed = 0;
            if (xSpeedController.atGoal())
                xSpeed = 0;

            swerve.drive(new ChassisSpeeds(xSpeed, 0, omegaSpeed));
        }
    }

    @Override
    public void end(boolean interrupted) {
        // This method will be called once when the command ends
        swerve.stop(); // Stop the robot when the command ends
    }

    double convertToRadians(double customUnitAngle) {
        return (customUnitAngle / 256.0) * Math.PI;
    }
}
