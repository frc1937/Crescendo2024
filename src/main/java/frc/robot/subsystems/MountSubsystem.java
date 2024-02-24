package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MountSubsystem extends SubsystemBase {
    private final DigitalInput proximitySwitch;
    private final WPI_TalonSRX mountMotor;
    private boolean proxyWasEnabled;

    public MountSubsystem() {
        proximitySwitch = new DigitalInput(1);
        mountMotor  = new WPI_TalonSRX(4);
        proxyWasEnabled = false;

        mountMotor.configFactoryDefault();
        mountMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ProximityStatus", getProxyStatus());
        SmartDashboard.putBoolean("WasProxyEnabled", !proxyWasEnabled);
    }

    public void startMount() {
        if(getProxyStatus() && proxyWasEnabled == false) {
            mountMotor.set(-0.1);
            proxyWasEnabled = true;
        } else if(getProxyStatus() && proxyWasEnabled) {
            proxyWasEnabled = false;
            stopMount();
        }
    }

    public void stopMount() {
        mountMotor.stopMotor();
    }

    public boolean getProxyStatus() {
        return proximitySwitch.get();
    }
}
