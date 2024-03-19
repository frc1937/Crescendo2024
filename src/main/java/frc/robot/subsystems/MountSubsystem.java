package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.Mount.*;

public class MountSubsystem extends SubsystemBase {
    private final CANSparkMax mountRightMotor = new CANSparkMax(MOUNT_RIGHT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax mountLeftMotor = new CANSparkMax(MOUNT_LEFT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private final RelativeEncoder rightEncoder = mountRightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = mountLeftMotor.getEncoder();

    public MountSubsystem() {
        configMotor(mountRightMotor);
        configMotor(mountLeftMotor);

        mountLeftMotor.setInverted(true);

        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("mount/rightEncoder", rightEncoder.getPosition());
        SmartDashboard.putNumber("mount/leftEncoder", leftEncoder.getPosition());
    }

    public boolean isAtTop() {
        return rightEncoder.getPosition() >= MOUNT_AT_TOP_LEFT_VALUE.in(Rotations) &&
                leftEncoder.getPosition() >= MOUNT_AT_TOP_RIGHT_VALUE.in(Rotations);
    }

    /**
    @param speed - Speed of the motors, ought to be positive
     */
    public void autoMount(double speed) {
        speed = Math.abs(speed);

        if(isAtTop()) {
            speed *= -1;
        }

        mountLeftMotor.set(speed);
        mountRightMotor.set(speed);
    }

    public void manualMount(double leftSpeed, double rightSpeed) {
        if(rightEncoder.getPosition() < MOUNT_AT_TOP_RIGHT_VALUE.in(Rotations))
            mountLeftMotor.set(leftSpeed);

        if(leftEncoder.getPosition() < MOUNT_AT_TOP_LEFT_VALUE.in(Rotations))
            mountRightMotor.set(rightSpeed);
    }

    public void stopMount() {
        mountLeftMotor.stopMotor();
        mountRightMotor.stopMotor();
    }

    private void configMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

//        motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) MOUNT_SOFT_LIMIT.in(Rotations));
//        motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    }
}
