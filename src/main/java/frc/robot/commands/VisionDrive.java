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
    private static final double ANGLE_THRESHOLD = 30.0;
    private static final double DISTANCE_THRESHOLD = 0.01;
    private int zeroDistanceCounter = 0;

    private final Swerve swerve;

    private NetworkTable visionTable;
    private NetworkTableEntry AngleEntry;
    private NetworkTableEntry DistanceEntry;
    private double Angle;
    private double Distance;

    private Timer timer;

    public VisionDrive(Swerve swerve) {
        this.swerve = swerve;
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        timer = new Timer();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Initialize the NetworkTable and entries here
        AngleEntry = visionTable.getEntry("Angle");
        DistanceEntry = visionTable.getEntry("Distance");
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        Angle = convertToRadians(AngleEntry.getDouble(0.0));
        Distance = DistanceEntry.getDouble(0.0);

        if (Distance > DISTANCE_THRESHOLD) {
            if (Math.abs(Angle) < ANGLE_THRESHOLD) {
                swerve.drive(new ChassisSpeeds(Distance / 2, 0, 0));
            } else {
                if (timer.get() == 0.0) {
                    timer.reset();
                    timer.start();
                }

                if (timer.get() < 1.0) {
                    swerve.drive(new ChassisSpeeds(0, 0, Angle));
                } else {
                    timer.stop();
                    swerve.drive(new ChassisSpeeds(0, 0, 0));
                }

                if (Math.abs(Angle) < ANGLE_THRESHOLD) {
                    // Reset the timer: you want to run the rotation command again
                    timer.reset();
                }
            }
        } else {
            zeroDistanceCounter++;
            if (zeroDistanceCounter >= MAX_ZERO_DISTANCE_COUNT) {
                return;
            }
            zeroDistanceCounter = 0;
        }

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
        return DistanceEntry.getDouble(0.0) < DISTANCE_THRESHOLD;
    }

    double convertToRadians(double customUnitAngle) {
        return (customUnitAngle / 256.0) * Math.PI;
    }
}
