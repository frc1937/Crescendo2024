package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionDataReceiver {
    public static void main(String[] args) {
        // Set the network tables server address
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("localhost");

        // Get the "Vision" table
        NetworkTable table = inst.getTable("Vision");

        // Continuously read data from NetworkTables
        while (true) {
            // Fetch and print the values in real-time
            for (String key : table.getKeys()) {
                double value = table.getEntry(key).getDouble(0.0);
                System.out.println(key + ": " + value);
            }

            // Add a delay of 1/30 seconds
            try {
                Thread.sleep(33);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
