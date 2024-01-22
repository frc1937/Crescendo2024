package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommands {
    IntakeSubsystem intake = new IntakeSubsystem();
    public Command startIntakeMotor(double speed) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    intake.setSpeedPercentage(speed);
                },
                interrupted -> {
                    stopIntakeMotor();
                },
                () -> false,
                intake
        );
    }

    public Command stopIntakeMotor() {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    intake.stopMotor();
                },
                interrupted -> {
                },
                () -> false,
                intake
        );
    }
}
