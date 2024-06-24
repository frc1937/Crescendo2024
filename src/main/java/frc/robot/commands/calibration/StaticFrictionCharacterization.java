//package frc.robot.commands.calibration;
//
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.shooter.Pitch;
//
//import java.util.function.DoubleSupplier;
//
//public class StaticFrictionCharacterization extends Command {
//    private DoubleSupplier supplier;
//    private Pitch pitch;
//    private double volts;
//
//    public StaticFrictionCharacterization(Pitch pitch) {
//        this.pitch = pitch;
//        SmartDashboard.putNumber("pitchVolts", 0);
//    }
//
//    @Override
//    public void execute() {
//        pitch.setRawVoltage(SmartDashboard.getNumber("pitchVolts", 0));
//    }
//}
