package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedSubsystem;


public class LedsCommand extends Command {
    private final LedSubsystem ledSubsystem;
    private final int r, g, b;
    public LedsCommand(LedSubsystem ledSubsystem, int r, int g, int b) {
        this.ledSubsystem = ledSubsystem;
        this.r = r;
        this.g = g;
        this.b = b;

        addRequirements(this.ledSubsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.startLeds(r, g, b);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
