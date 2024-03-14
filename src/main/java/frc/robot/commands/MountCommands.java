package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.MountSubsystem;

public class MountCommands {
    private final MountSubsystem mountSubsystem;

    public MountCommands(MountSubsystem mountSubsystem) {
        this.mountSubsystem = mountSubsystem;
    }

    public Command startMount(double leftSpeed, double rightSpeed) {
        return new FunctionalCommand(
                () -> mountSubsystem.setMount(leftSpeed, rightSpeed),
                () -> {
                },
                (interrupt) -> mountSubsystem.stopMount(),
                () -> false,
                mountSubsystem
        );
    }

    //It is in this format as we need to create multiple commands with different speeds.
}
