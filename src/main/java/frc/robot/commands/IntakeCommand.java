package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setSpeedPercentage(speed);
    } //todo: test if still works

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }
}
