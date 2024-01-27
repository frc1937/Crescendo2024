package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
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

    private Timer timer;
    private static final double DRIVE_DURATION = 1.0; // Set the duration in seconds

    public VisionDrive(Swerve swerve) {
        this.swerve = swerve;
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        timer = new Timer();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Initialize the NetworkTable and entries here
        AngleEntry = visionTable.getEntry("Angle");
        DistanceEntry = visionTable.getEntry("Distance");
    
        while (DistanceEntry.getDouble(0.0) != 0.0) {
            // Update angle and distance
            Angle = convertToRadians(AngleEntry.getDouble(0.0));
            Distance = DistanceEntry.getDouble(0.0);
    
            // Check if Distance is zero
            if (Distance == 0.0) {
                zeroDistanceCounter++;
    
                // Check if zeroDistanceCounter exceeds the threshold
                if (zeroDistanceCounter >= MAX_ZERO_DISTANCE_COUNT) {
                    break; // Exit the loop if consecutive zero distances exceed the threshold
                }
            } else {
                // Reset the counter if Distance is not zero
                zeroDistanceCounter = 0;
            }
    
            // Check if the angle is close to zero before rotating
            if (Math.abs(Angle) < 0.05) {
                swerve.drive(new ChassisSpeeds(Distance / 2, 0, 0));
            } else {
                // Rotate to correct angle
                swerve.drive(new ChassisSpeeds(0, 0, Angle));
    
                // Check if the angle is close to zero after correcting rotation (0-30 is a good balance)
                if (Math.abs(Angle) < 30) {
                    // Move forward if the angle is close to zero
                    swerve.drive(new ChassisSpeeds(Distance / 2, 0, 0));
                }
            }
    
            Timer.delay(0.03);
        }
    
        // Drive the robot forward
        swerve.drive(new ChassisSpeeds(Distance / 2, 0, 0));
    }
    
    @Override
    public void end(boolean interrupted) {
        // This method will be called once when the command ends
        swerve.stop(); // Stop the robot when the command ends
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        Distance = DistanceEntry.getDouble(0.0);
        return Distance < 0.01
    }

    double convertToRadians(double customUnitAngle) {
        return (customUnitAngle / 256.0) * Math.PI;
    }
    
}
