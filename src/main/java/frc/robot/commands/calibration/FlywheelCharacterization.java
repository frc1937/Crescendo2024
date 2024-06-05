package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class FlywheelCharacterization {
    private final SysIdRoutine routine;

    public FlywheelCharacterization(ShooterSubsystem shooter) {
        this.routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.8).per(Second),
                        Volts.of(6),
                        Seconds.of(10)
                ),
                new SysIdRoutine.Mechanism(shooter::setFlywheelVoltage, shooter::logFlywheels, shooter)
        );
    }

    public Command sysIdQuastaticTest(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamicTest(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
