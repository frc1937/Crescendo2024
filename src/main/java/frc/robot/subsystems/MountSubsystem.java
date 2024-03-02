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
        SmartDashboard.putNumber("ProximityStatus", getProximityStatus());
    }

    public void startMount() {
        mountMotor.set(0.5);
    }
    // WARNING: THIS METHOD IS FOR PIT RESET, DO NOT USE ON A REAL GAME.
    public void startMountBackwards() {
        mountMotor.set(-0.5);
    }

    public void stopMount() {
        mountMotor.stopMotor();
    }

    /**
     * @return 0 if the mount subsystem is in a final state or 1 in an
     * intermediate state
     */
    public int getProximityStatus() {
        return proximitySwitch.get() ? 1 : 0;
    }
}
