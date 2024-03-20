package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.Mount.*;

public class MountSubsystem extends SubsystemBase {
    private final CANSparkMax rightMotor = new CANSparkMax(MOUNT_RIGHT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax leftMotor = new CANSparkMax(MOUNT_LEFT_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

    public MountSubsystem() {
        configMotor(rightMotor);
        configMotor(leftMotor);

        leftMotor.setInverted(true);

        rightEncoder.setPosition(0);
        leftEncoder.setPosition(0);

        leftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) MOUNT_AT_TOP_LEFT_VALUE.in(Rotations));
        leftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        rightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) MOUNT_AT_TOP_RIGHT_VALUE.in(Rotations));
        rightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
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

    public void manualMount(double leftSpeed, double rightSpeed) {
        leftMotor.set(leftSpeed);

        rightMotor.set(rightSpeed);
    }

    public void stopMount() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    private void configMotor(CANSparkMax motor) {
       motor.restoreFactoryDefaults();
       motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

       motor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0.f);
       motor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    }
}
