package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.IntakeConstants.INTAKE_MOTOR_ID;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonSRX talonSRX = new WPI_TalonSRX(INTAKE_MOTOR_ID);

    public IntakeSubsystem(BooleanSupplier isInverted) {
        talonSRX.setInverted(isInverted.getAsBoolean());
        //a button to set the boolean
    }

    public void setSpeedPercentage(double percentage) {
        talonSRX.set(ControlMode.PercentOutput, percentage);
    }

    public void stopMotor() {
        talonSRX.stopMotor();
    }
}
