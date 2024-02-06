package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double speed) {
        this.speed = speed;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.setSpeedPercentage(speed);
    }


    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotor();
    }
}
