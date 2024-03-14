package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.MountSubsystem;

import java.util.function.DoubleSupplier;

public class MountCommands {
    private final MountSubsystem mountSubsystem;

    public MountCommands(MountSubsystem mountSubsystem) {
        this.mountSubsystem = mountSubsystem;
    }

    public Command startManualMount(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
        return new FunctionalCommand(
                () -> mountSubsystem.manualMount(leftSpeed.getAsDouble(), rightSpeed.getAsDouble()),
                () -> {
                },
                (interrupt) -> mountSubsystem.stopMount(),
                () -> false,
                mountSubsystem
        );
    }

    public Command startAutomaticMount(double speed) {
        return new FunctionalCommand(
                () -> mountSubsystem.autoMount(speed),
                () -> {},
                (interrupt) -> mountSubsystem.stopMount(),
                () -> false,
                mountSubsystem
        );
    }

    //It is in this format as we need to create multiple commands with different speeds.
}
