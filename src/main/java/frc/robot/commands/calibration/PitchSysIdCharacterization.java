package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.*;

public class PitchSysIdCharacterization implements SysIdCharacterization {

    private final SysIdRoutine routine;

    public PitchSysIdCharacterization(ShooterSubsystem shooterSubsystem) {
        SysIdRoutine.Config config = new SysIdRoutine.Config(
                Volts.per(Second).of(0.5),
                Volts.of(3),
                Seconds.of(10)
        );

        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                shooterSubsystem::setPitchVoltage,
                shooterSubsystem::logPitch,
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

