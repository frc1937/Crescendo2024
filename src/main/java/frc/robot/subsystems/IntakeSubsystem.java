package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IntakeSubsystem extends SubsystemBase {

    private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.IntakeConstants.IntakeMotorPort);

    public IntakeSubsystem() {
        // Replace 1 with the CAN ID of the actual TalonSRX motor controller
    }

    public void setSpeed(double speed) {
        // Set the motor speed (positive or negative) to make the engine rotate
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    // Stop the motor by setting the output to 0
    public void stopMotor() {
        intakeMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called automatically, add any periodic tasks here
    }
}
