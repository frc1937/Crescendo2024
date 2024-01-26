package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class VisionDrive extends Command {
    
    private static final int MAX_ZERO_DISTANCE_COUNT = 3;
    private int zeroDistanceCounter = 0;

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

    }

    @Override
    public void execute() {
        // Initialize the NetworkTable and entries here
        AngleEntry = visionTable.getEntry("Angle");
        DistanceEntry = visionTable.getEntry("Distance");
        Angle = AngleEntry.getDouble(0.0);
        Distance = DistanceEntry.getDouble(0.0);
        // Check if Distance is zero
        if (Distance == 0.0) {
            zeroDistanceCounter++;

            // Check if zeroDistanceCounter exceeds the threshold
            if (zeroDistanceCounter >= MAX_ZERO_DISTANCE_COUNT) {
                end(false); // Ending the command with interrupted = false
                return;
            }
        } else {
            // Reset the counter if Distance is not zero
            zeroDistanceCounter = 0;
        }

        if (Angle > 0.15 || Angle < 0.15) {
            swerve.drive(new ChassisSpeeds(0 , 0, Angle));
            }

        // Drive the robot
        swerve.drive(new ChassisSpeeds(Distance / 2 , 0, 0));

    }

    @Override
    public void end(boolean interrupted) {
        // This method will be called once when the command ends
        swerve.stop();  // Stop the robot when the command ends
    }

    @Override
    public boolean isFinished() {
        Distance = DistanceEntry.getDouble(0.0);
        return Distance < 0.01;
    }
}
