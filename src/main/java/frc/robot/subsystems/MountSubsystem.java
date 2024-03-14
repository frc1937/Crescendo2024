package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.Mount.*;

public class MountSubsystem extends SubsystemBase {
    private final CANSparkMax mountRightMotor = new CANSparkMax(MOUNT_RIGHT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax mountLeftMotor = new CANSparkMax(MOUNT_LEFT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private final RelativeEncoder encoder = mountLeftMotor.getEncoder();

    public MountSubsystem() {
        configMotor(mountRightMotor);
        configMotor(mountLeftMotor);
    }

    public boolean isAtTop() {
        return encoder.getPosition() >= MOUNT_AT_TOP_ENCODER_VALUE.in(Rotations);
    }

    /**
    @param speed - Speed of the motors, ought to be positive
     */
    public void autoMount(double speed) {
        speed = Math.abs(speed);

        if(encoder.getPosition() >= MOUNT_AT_TOP_ENCODER_VALUE.in(Rotations)) {
            speed *= -1;
        }

        mountLeftMotor.set(speed);
        mountRightMotor.set(speed);
    }

    public void manualMount(double leftSpeed, double rightSpeed) {
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

        //todo: check directions on ROBOT
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) MOUNT_SOFT_LIMIT.in(Rotations));
    }
}
