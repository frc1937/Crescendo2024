package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.GlobalConstants.CanIDConstants.INTAKE_MOTOR_ID;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonSRX motor = new WPI_TalonSRX(INTAKE_MOTOR_ID);

    public IntakeSubsystem() {
        configureMotor();
    }

    public void setSpeedPercentage(double percentage) {
        motor.set(ControlMode.PercentOutput, percentage);
    }

    public void stop() {
        motor.stopMotor();
    }

    private void configureMotor() {
        motor.configFactoryDefault();

        motor.setNeutralMode(NeutralMode.Coast);
        motor.setInverted(true);
    }
}
