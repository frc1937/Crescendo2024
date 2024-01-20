package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Intake extends SubsystemBase {

    private final TalonSRX engineMotor;

    public Intake() {
        // Replace 1 with the CAN ID of your TalonSRX motor controller
        engineMotor = new TalonSRX(1);
    }

    public void spin(double speed) {
        // Set the motor speed (positive or negative) to make the engine spin
        engineMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        // Stop the motor by setting the output to 0
        engineMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        // This method will be called automatically, add any periodic tasks here
    }
}
