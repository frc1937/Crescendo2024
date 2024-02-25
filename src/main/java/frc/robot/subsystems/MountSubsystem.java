package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MountSubsystem extends SubsystemBase {
    private final DigitalInput proximitySwitch;
    private final WPI_TalonSRX mountMotor;
    private boolean isRunning;

    public MountSubsystem() {
        proximitySwitch = new DigitalInput(1);
        mountMotor  = new WPI_TalonSRX(4);
        isRunning = false;

        mountMotor.configFactoryDefault();
        mountMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        int proxyValue = getProxyStatus();

        SmartDashboard.putBoolean("ProximityStatus", getProxyStatus() == 1);
        SmartDashboard.putBoolean("WasProxyEnabled", !isRunning);

        if(proxyValue == 1 && !isRunning) {
            startMount();
            isRunning = true;
        } else if (proxyValue == 1 && isRunning == true) {
            stopMount();
            isRunning = false;
        }
    }

    public void startMount() {
        mountMotor.set(-0.1);
    }

    public void stopMount() {
        mountMotor.stopMotor();
    }

    public int getProxyStatus() {
        return proximitySwitch.get() ? 1 : 0;
    }
}
