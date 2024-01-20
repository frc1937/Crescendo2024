package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class VisionDrive extends Command {

    private final Vision vision;
    private final Swerve swerve;

    public VisionDrive(Vision visionSubsystem, Swerve swerveSubsystem) {
        this.vision = visionSubsystem;
        this.swerve = swerveSubsystem;
    
        // Add null checks before calling addRequirements
        if (visionSubsystem != null && swerveSubsystem != null) {
            addRequirements(visionSubsystem, swerveSubsystem);
        } else {
            // System.out.println("Error: One or more subsystems are null.");
        }
    }
    

    @Override
    public void initialize() {
        // This method will be called once when the command is scheduled to run
    }

    @Override
    public void execute() {
        // This method will be called repeatedly while the command is scheduled to run

        // Fetch vision data
        double distance = vision.getDistance();
        double angle = vision.getAngle();

        // Implement logic to drive towards the object based on vision data
        double forwardSpeed = calculateForwardSpeed(distance);
        double turnSpeed = calculateTurnSpeed(angle);

        // Drive the robot
        swerve.drive(new Translation2d(forwardSpeed, 0), turnSpeed, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        // This method will be called once when the command ends
        swerve.stop(); // Stop the robot when the command ends
    }

    private double calculateForwardSpeed(double distance) {
        return 0.1 * distance; // Placeholder, adjust as needed
    }

    private double calculateTurnSpeed(double angle) {
        return 0.01 * angle; // Placeholder, adjust as needed
    }
}
