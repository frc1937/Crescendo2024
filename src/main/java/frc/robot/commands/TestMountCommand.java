package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.MountSubsystem;


public class TestMountCommand {
    private final MountSubsystem mountSubsystem;

    public TestMountCommand(MountSubsystem mountSubsystem) {
        this.mountSubsystem = mountSubsystem;
    }

    public FunctionalCommand mountCommand() {
        return new FunctionalCommand(
                mountSubsystem::startMount,
                () -> {},
                interrupted -> mountSubsystem.stopMount(),
                () -> mountSubsystem.getProximityStatus() == 0,
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
