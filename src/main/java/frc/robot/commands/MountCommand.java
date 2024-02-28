package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.MountSubsystem;


public class MountCommand {
    private final MountSubsystem mountSubsystem;

    public MountCommand(MountSubsystem mountSubsystem) {
        this.mountSubsystem = mountSubsystem;
    }

    public FunctionalCommand mountCommand() {
        return new FunctionalCommand(
                mountSubsystem::startMount,
                () -> {},
                interrupt -> mountSubsystem.stopMount(),
                () -> false,
                mountSubsystem
        );
    }

    public FunctionalCommand testMountCommand() {
        return new FunctionalCommand(
                mountSubsystem::startMountBackwards,
                () -> {},
                interrupt -> mountSubsystem.stopMount(),
                () -> false,
                mountSubsystem
        );
    }

}
