package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MountSubsystem;

public class Mount extends Command {
    private final MountSubsystem mount;
    private boolean reachedOutOfProximity;

    public Mount(MountSubsystem mount) {
        this.mount = mount;

        addRequirements(mount);
    }

    @Override
    public void initialize() {
        mount.startMount();
        reachedOutOfProximity = mount.getProximityStatus() == 1;
    }

    @Override
    public void execute() {
        reachedOutOfProximity = reachedOutOfProximity || mount.getProximityStatus() == 1;
    }

    @Override
    public void end(boolean interrupted) {
        mount.stopMount();
    }

    @Override
    public boolean isFinished() {
        return reachedOutOfProximity && mount.getProximityStatus() == 0;
    }
}
