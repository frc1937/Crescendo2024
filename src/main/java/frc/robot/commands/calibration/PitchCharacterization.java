package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class PitchCharacterization {

    private final SysIdRoutine routine;

    public PitchCharacterization(ShooterSubsystem shooterSubsystem) {
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

    public Command sysIdQuastaticTest(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamicTest(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}

