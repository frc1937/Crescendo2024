//package frc.robot.commands.calibration;
//
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.shooter.Pitch;
//
//
//public class GearRatioCharacterization extends Command {
//    private final double duration;
//    private final double voltage;
//
//    /**
//     * Both are in rotations. With NO scaling applied.
//     */
//    private double externalMeasure, internalMeasure;
//    private double startTimestamp;
//
//    private final Pitch pitch;
//
//    public GearRatioCharacterization(Pitch pitch, double voltage, double duration) {
//        this.voltage = voltage;
//        this.duration = duration;
//        this.pitch = pitch;
//    }
//
//    @Override
//    public void initialize() {
//        pitch.setRawVoltage(voltage);
//
//        externalMeasure = pitch.getPosition().getDegrees();
//        internalMeasure = pitch.getRelativeEncoderPosition() * 360;
//
//        startTimestamp = Timer.getFPGATimestamp();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return Timer.getFPGATimestamp() - startTimestamp >= duration;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        pitch.setRawVoltage(0.0);
//
//        final double externalAccumulation = pitch.getPosition().getDegrees() - externalMeasure;
//        final double internalAccumulation = pitch.getRelativeEncoderPosition() * 360 - internalMeasure;
//
//        SmartDashboard.putNumber("Characterization/GearRatio/RATIOExternalToInternal", externalAccumulation / internalAccumulation);
//        SmartDashboard.putNumber("Characterization/GearRatio/RATIOInternalToExternal", internalAccumulation / externalAccumulation);
//        SmartDashboard.putNumber("Characterization/GearRatio/ExternalValue", externalAccumulation);
//        SmartDashboard.putNumber("Characterization/GearRatio/InternalValue", internalAccumulation);
//    }
//}
////147.847088
////147.978402