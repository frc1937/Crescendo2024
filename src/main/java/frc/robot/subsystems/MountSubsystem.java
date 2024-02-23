package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MountSubsystem extends SubsystemBase {
    private final DigitalInput proximitySwitch = new DigitalInput(1);
    private final WPI_TalonSRX mountMotor = new WPI_TalonSRX(4);
    private boolean shouldStartMotor;
    private boolean proxyWasEnabled = false;

    public MountSubsystem() {
        mountMotor.configFactoryDefault();
        mountMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Mount Status", proximitySwitch.get());

        if(getProxyStatus() && shouldStartMotor && !proxyWasEnabled) {
            mountMotor.set(0.1);
            proxyWasEnabled = true;
        }

        if(getProxyStatus() && proxyWasEnabled) {
            proxyWasEnabled = false;
            stopMount();
        }
    }

    public void startMount() {
        shouldStartMotor = true;
    }

    public void stopMount() {
        mountMotor.stopMotor();
        shouldStartMotor = false;
    }

    public boolean getProxyStatus() {
        return proximitySwitch.get();
    }
    //It's 1 when it's up, 1 when it's down, 0 in between.
}
