package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand {
    private final IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
    }

    public Command startIntakeMotor(double speed) {
        return new FunctionalCommand(
                () -> {
                    // Initialization code if needed
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
                () -> {
                    // Cleanup code if needed
                },
                () -> {
                    intake.stopMotor();
                },
                interrupted -> {
                    // Interrupted code if needed
                },
                () -> false,
                intake
        );
    }
}
