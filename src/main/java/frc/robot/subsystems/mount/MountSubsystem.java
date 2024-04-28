package frc.robot.subsystems.mount;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.Mount.*;

public class MountSubsystem extends SubsystemBase {
    private final CANSparkMax rightMotor = new CANSparkMax(MOUNT_RIGHT_MOTOR_ID, MotorType.kBrushless);
    private final CANSparkMax leftMotor = new CANSparkMax(MOUNT_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    public MountSubsystem() {
        configMotor(rightMotor);
        configMotor(leftMotor);

        leftMotor.setInverted(true);

        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);

        leftMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MOUNT_AT_TOP_LEFT_VALUE.in(Rotations));
        leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        rightMotor.setSoftLimit(SoftLimitDirection.kForward, (float) MOUNT_AT_TOP_RIGHT_VALUE.in(Rotations));
        rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("mount/rightEncoder", rightEncoder.getPosition());
        SmartDashboard.putNumber("mount/leftEncoder", leftEncoder.getPosition());
    }

    public void manualMount(double leftSpeed, double rightSpeed) {
        leftMotor.set(leftSpeed);

        rightMotor.set(rightSpeed);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    private void configMotor(CANSparkMax motor) {
       motor.restoreFactoryDefaults();
       motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

       motor.setSoftLimit(SoftLimitDirection.kReverse, 0.f);
       motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }
}
