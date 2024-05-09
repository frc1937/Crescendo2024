package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class IntakeCommands {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeCommands(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command enableIntake(double speed, boolean shouldStop) {
        return new FunctionalCommand(
                () -> intakeSubsystem.setSpeedPercentage(speed),
                () -> {
                },
                interrupted -> {
                    if (shouldStop)
                        intakeSubsystem.stop();
                },
                () -> false,

                intakeSubsystem
        );
    }

    public Command stopIntake(double delay) {
        return new WaitCommand(delay).andThen(new FunctionalCommand(
                intakeSubsystem::stop,
                () -> {
                },
                interrupted -> {
                },
                () -> false,

                intakeSubsystem
        ));
    }
}