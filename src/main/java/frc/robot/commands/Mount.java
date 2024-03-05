package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MountSubsystem;

public class Mount extends Command {
    private final MountSubsystem mountSubsystem;
    private boolean reachedOutOfProximity;

    public Mount(MountSubsystem mountSubsystem) {
        this.mountSubsystem = mountSubsystem;

        addRequirements(mountSubsystem);
    }

    @Override
    public void initialize() {
        mountSubsystem.startMount();
        reachedOutOfProximity = mountSubsystem.getProximityStatus() == 1;
    }

    @Override
    public void execute() {
        reachedOutOfProximity = reachedOutOfProximity || mountSubsystem.getProximityStatus() == 1;
    }

    @Override
    public void end(boolean interrupted) {
        mountSubsystem.stopMount();
    }

    @Override
    public boolean isFinished() {
        return reachedOutOfProximity && mountSubsystem.getProximityStatus() == 0;
    }
}
