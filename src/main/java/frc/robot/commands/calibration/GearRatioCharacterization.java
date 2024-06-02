package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Pitch;


public class GearRatioCharacterization extends Command {
    private final double duration;
    private final double voltage;

    /**
     * Both are in rotations. With NO scaling applied.
     */
    private double externalMeasure, internalMeasure;
    private double startTimestamp;

    private final Pitch pitch;

    public GearRatioCharacterization(Pitch pitch, double voltage, double duration) {
        this.voltage = voltage;
        this.duration = duration;
        this.pitch = pitch;
    }

    @Override
    public void initialize() {
        pitch.drivePitch(voltage);

        externalMeasure = pitch.getPosition().getRotations();
        internalMeasure = pitch.getRelativeEncoderPosition();

        startTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTimestamp >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        pitch.drivePitch(0.0);

        final double externalAccumulation = pitch.getPosition().getRotations() - externalMeasure;
        final double internalAccumulation = pitch.getPosition().getRotations() - internalMeasure;

        SmartDashboard.putNumber("GearRatioCharacterization/ExternalToInternalRatio", externalAccumulation / internalAccumulation);
        SmartDashboard.putNumber("GearRatioCharacterization/ExternalValue", externalAccumulation);
        SmartDashboard.putNumber("GearRatioCharacterization/InternalValue", internalAccumulation);
    }
}
