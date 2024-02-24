package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MountSubsystem;


public class MountCommand extends Command {
    private final MountSubsystem mountSubsystem;

    public MountCommand(MountSubsystem mountSubsystem) {
        this.mountSubsystem = mountSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.mountSubsystem);
    }

    @Override
    public void initialize() {
        mountSubsystem.startMount();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mountSubsystem.stopMount();
    }
}
