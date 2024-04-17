package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.INTAKE_MOTOR_ID;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonSRX talonSRX = new WPI_TalonSRX(INTAKE_MOTOR_ID);

    public IntakeSubsystem() {
        configureTalonMotor(talonSRX);
    }

    public void setSpeedPercentage(double percentage) {
        talonSRX.set(ControlMode.PercentOutput, percentage);
    }

    public void stop() {
        talonSRX.stopMotor();
    }

    private void configureTalonMotor(WPI_TalonSRX motor) {
        motor.configFactoryDefault();

        motor.setNeutralMode(NeutralMode.Coast);
        talonSRX.setInverted(true);
    }
}
