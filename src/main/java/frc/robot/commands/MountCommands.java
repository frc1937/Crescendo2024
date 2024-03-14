package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.MountSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Mount.MOUNT_AUTO_SPEED;
import static frc.robot.Constants.Mount.MOUNT_SPEED_SCALAR;

public class MountCommands {
    private final MountSubsystem mountSubsystem;

    public MountCommands(MountSubsystem mountSubsystem) {
        this.mountSubsystem = mountSubsystem;
    }

    public Command startManualMount(DoubleSupplier leftSpeedSup, DoubleSupplier rightSpeedSup) {
        return new FunctionalCommand(
                () -> mountSubsystem.manualMount(
                        leftSpeedSup.getAsDouble() * MOUNT_SPEED_SCALAR, rightSpeedSup.getAsDouble() * MOUNT_SPEED_SCALAR
                ),
                () -> {
                },
                interrupt -> mountSubsystem.stopMount(),
                () -> false,

                mountSubsystem
        );
    }

    public Command startAutomaticMount() {
        return new FunctionalCommand(
                () -> mountSubsystem.autoMount(MOUNT_AUTO_SPEED),
                () -> {},
                interrupt -> mountSubsystem.stopMount(),
                mountSubsystem::isAtTop,

                mountSubsystem
        );
    }
}
