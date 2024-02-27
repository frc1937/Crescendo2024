package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MountSubsystem extends SubsystemBase {
    private final DigitalInput proximitySwitch;
    private final WPI_TalonSRX mountMotor;
    private boolean shouldMountRun = false;

    public MountSubsystem() {
        proximitySwitch = new DigitalInput(1);
        mountMotor  = new WPI_TalonSRX(4);

        mountMotor.configFactoryDefault();
        mountMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        int proxyValue = getProxyStatus();

        SmartDashboard.putBoolean("ProximityStatus", getProxyStatus() == 1); //THIS CONDITION IS FALSE AT TOP AND BOTTOM,
        // ELSE TRUE

        if(proxyValue == 0 && shouldMountRun) {
            startMount();
            shouldMountRun = false;
        } else if (proxyValue == 0) {
            stopMount();
            shouldMountRun = true;
        }
    }

    public void startMount() {
        mountMotor.set(-0.5);
        shouldMountRun = true;
    }
    //THIS METHOD IS FOR PIT RESET, DO NOT USE ON A REAL GAME.
    public void startMountBackwards() {
        mountMotor.set(0.5);
        shouldMountRun = false;
    }

    public void stopMount() {
        mountMotor.stopMotor();
    }

    public int getProxyStatus() {
        return proximitySwitch.get() ? 1 : 0;
    }
}
