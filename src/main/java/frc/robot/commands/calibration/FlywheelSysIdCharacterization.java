package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.*;

public class FlywheelSysIdCharacterization implements SysIdCharacterization {
    private final SysIdRoutine routine;

    public FlywheelSysIdCharacterization(ShooterSubsystem shooterSubsystem) {
        SysIdRoutine.Config config = new SysIdRoutine.Config(
                Volts.of(0.8).per(Second),
                Volts.of(6),
                Seconds.of(10)
        );

        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                shooterSubsystem::setFlywheelVoltage,
                shooterSubsystem::logFlywheels,
                shooterSubsystem
        );

        routine  = new SysIdRoutine(
                config,
                mechanism
        );
    }


    @Override
    public SysIdRoutine getSysIdRoutine() {
        return routine;
    }
}
