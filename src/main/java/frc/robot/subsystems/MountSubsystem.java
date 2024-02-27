package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MountSubsystem extends SubsystemBase {
    private final DigitalInput proximitySwitch;
    private final WPI_TalonSRX mountMotor;
    private int isRunning;

    public MountSubsystem() {
        proximitySwitch = new DigitalInput(1);
        mountMotor  = new WPI_TalonSRX(4);
        isRunning = 0;

        mountMotor.configFactoryDefault();
        mountMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        int proxyValue = getProxyStatus();

        SmartDashboard.putBoolean("ProximityStatus", getProxyStatus() == 1);
        SmartDashboard.putNumber("WasProxyEnabled", isRunning);

        if(proxyValue == 1 && isRunning < 5) {
            startMount();
            isRunning++;
        } else if (proxyValue == 1 && isRunning >= 5) {
            stopMount();
            isRunning = 0;
        }
    }

    public void startMount() {
        mountMotor.set(-0.1);
    }
    //THIS METHOD IS FOR PIT RESET, DO NOT USE ON A REAL GAME.
    public void startMountBackwards() {
        mountMotor.set(0.1);
    }

    public void stopMount() {
        mountMotor.stopMotor();
    }

    public int getProxyStatus() {
        return proximitySwitch.get() ? 1 : 0;
    }
}
