package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MountSubsystem extends SubsystemBase {
    private final CANSparkMax mountRightMotor = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax mountLeftMotor = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);

    //todo:
    // Move magic numbers to constants
    // use encoders to determine direction of movement on AUTO-MOUNT
    // add buttons for AUTO-MOUNT(Determine direction, move both upwards together), and MANUAL-MOUNT(Left-Right omnidirectional control)

    public MountSubsystem() {
        configMotor(mountRightMotor);
        configMotor(mountLeftMotor);
    }

    public void setMount(double leftSpeed, double rightSpeed) {
        mountLeftMotor.set(leftSpeed);
        mountRightMotor.set(rightSpeed);
    }

    public void stopMount() {
        mountLeftMotor.stopMotor();
        mountRightMotor.stopMotor();
    }

    private void configMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, 4);
    }
}
