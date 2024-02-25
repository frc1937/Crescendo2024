package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MountSubsystem;


public class MountCommand extends Command {
    private final MountSubsystem mountSubsystem;

    public MountCommand(MountSubsystem mountSubsystem) {
        this.mountSubsystem = mountSubsystem;

        addRequirements(this.mountSubsystem);
    }

    @Override
    public void initialize() {
        mountSubsystem.startMount();
    }

    @Override
    public void end(boolean interrupted) {
        mountSubsystem.stopMount();
    }
}
