package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.MountSubsystem;

import static frc.robot.Constants.Mount.MOUNT_AUTO_SPEED;

public class MountCommands {
    private final MountSubsystem mountSubsystem;

    public MountCommands(MountSubsystem mountSubsystem) {
        this.mountSubsystem = mountSubsystem;
    }

//    public Command startManualMount(DoubleSupplier leftSpeedSup, DoubleSupplier rightSpeedSup) {
//        return new FunctionalCommand(
//                () ->
//                ),
//                () -> {
//                },
//                interrupt -> ,
//                () -> false,
//
//                mountSubsystem
//        );
//    }

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
