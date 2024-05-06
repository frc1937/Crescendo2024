package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class IntakeCommands {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeCommands(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command enableIntake(double speed) {
        return new FunctionalCommand(
                () -> intakeSubsystem.setSpeedPercentage(speed),
                () -> {},
                interrupted -> {intakeSubsystem.stop();},
                () -> false,

                intakeSubsystem
        );
    }
}