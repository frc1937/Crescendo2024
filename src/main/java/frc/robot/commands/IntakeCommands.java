package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommands {
    private final IntakeSubsystem intakeSubsystem;
    public IntakeCommands(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }
    public Command startIntake(double speed) {
        return new FunctionalCommand(
                () -> {},
                () -> intakeSubsystem.setSpeedPercentage(speed),
                (interrupted) -> {},
                () -> false,
                intakeSubsystem
        );
    }

    public Command stopIntake() {
        return new FunctionalCommand(
                () -> {},
                intakeSubsystem::stopMotor,
                (interrupted) -> {},
                () -> false,
                intakeSubsystem
        );
    }
}
