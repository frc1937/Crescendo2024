package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;


public class GearRatioCharacterization extends Command {
    private final Consumer<Double> drive;
    private final double duration;
    private final double voltage;

    /**
     * Both are in rotations. With NO scaling applied.
     */
    private double externalPositionMeasure, internalPositionMeasurement;
    private double startTimestamp;

    private final DoubleSupplier externalPositionMeasurementSupplier, motorSupplier;

    public GearRatioCharacterization(Subsystem requirements, double voltage, Consumer<Double> drive, double duration,
                                     DoubleSupplier externalPositionMeasurementSupplier, DoubleSupplier motorSupplier) {
        this.voltage = voltage;
        this.drive = drive;
        this.duration = duration;
        this.externalPositionMeasurementSupplier = externalPositionMeasurementSupplier;
        this.motorSupplier = motorSupplier;
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        externalPositionMeasure = externalPositionMeasurementSupplier.getAsDouble();
        internalPositionMeasurement = motorSupplier.getAsDouble();

        drive.accept(voltage);

        startTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTimestamp >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        drive.accept(0.0);

        final double systemAccum = externalPositionMeasurementSupplier.getAsDouble() - externalPositionMeasure;
        final double motorAccum = motorSupplier.getAsDouble() - internalPositionMeasurement;

        SmartDashboard.putNumber("GearRatioCharacterization/SystemToMotor", systemAccum / motorAccum);
    }
}
