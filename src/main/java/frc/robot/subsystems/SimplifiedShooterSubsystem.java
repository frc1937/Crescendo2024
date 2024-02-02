package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

public class SimplifiedShooterSubsystem extends SubsystemBase {
    //no PID controller. just set power.
    private final CANSparkFlex flywheelMaster = new CANSparkFlex(FLYWHEEL_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex flywheelSlave = new CANSparkFlex(FLYWHEEL_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex pivotMotor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);

    public SimplifiedShooterSubsystem() {
        configureSparkMotor(flywheelMaster);
        configureSparkMotor(flywheelSlave);
        configureSparkMotor(pivotMotor);
    }

    public void setPivotPower(double power) {
        //SAFETY MEASUREMENTS SO ALON WON'T BE MAD
        if(Math.abs(power) > 0.2) return;

        pivotMotor.set(power);
    }

    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    public void startFlywheels() {
        flywheelMaster.set(0.7);
        flywheelSlave.set(0.7);
    }

    public void stopFlywheels() {
        flywheelMaster.stopMotor();
        flywheelSlave.stopMotor();
    }

    private void configureSparkMotor(CANSparkFlex motor) {
        motor.restoreFactoryDefaults();
    }
}
