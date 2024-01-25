package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class VisionDrive extends Command {
    
    private final Swerve swerve;

    private NetworkTable visionTable;
    private NetworkTableEntry AngleEntry;
    private NetworkTableEntry DistanceEntry;
    private double Angle; 
    private double Distance;

    public VisionDrive(Swerve swerve) {
        this.swerve = swerve;
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Initialize the NetworkTable and entries here
        AngleEntry = visionTable.getEntry("Angle");
        DistanceEntry = visionTable.getEntry("Distance");
        Angle = AngleEntry.getDouble(0.0);
        Distance = DistanceEntry.getDouble(0.0);
        
        // Check if Distance is zero, and end the command if true
        if (Distance == 0.0) {
            end(false); // Ending the command with interrupted = false
            return;
        }
        // Drive the robot
        swerve.drive(new ChassisSpeeds(Distance, 0, Angle));
    }

    @Override
    public void execute() {
        Angle = AngleEntry.getDouble(0.0);
        Distance = DistanceEntry.getDouble(0.0);

        // Check if Distance is zero, and end the command if true
        if (Distance == 0.0) {
            end(false); // Ending the command with interrupted = false
            return;
        }
        // Drive the robot
        swerve.drive(new ChassisSpeeds(Distance, 0, Angle));

    }

    @Override
    public void end(boolean interrupted) {
        // This method will be called once when the command ends
        swerve.stop();  // Stop the robot when the command ends
    }
}
