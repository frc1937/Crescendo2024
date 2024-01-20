package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private static final String TABLE_NAME = "Vision";
    private NetworkTable visionTable;
    private NetworkTableEntry distanceEntry;
    private NetworkTableEntry angleEntry;

    private double distance;
    private double angle;

    public Vision() {
        // Initialize the NetworkTable and entries here
        visionTable = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
        distanceEntry = visionTable.getEntry("distance");
        angleEntry = visionTable.getEntry("angle");
    }

    public void updateVisionData() {
        // Read values from NetworkTable entries
        distance = distanceEntry.getDouble(0.0); // Provide a default value: 0.0
        angle = angleEntry.getDouble(0.0); // Provide a default value: 0.0
    }

    public double getDistance() {
        return distance;
    }

    public double getAngle() {
        return angle;
    }
}
