package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MountSubsystem;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Mount.MOUNT_SPEED_SCALAR;


public class MountCommand extends Command {
    private final MountSubsystem mountSubsystem;
    private final DoubleSupplier leftSpeedSup, rightSpeedSup;

    public MountCommand(MountSubsystem mountSubsystem, DoubleSupplier leftSpeedSup, DoubleSupplier rightSpeedSup) {
        this.mountSubsystem = mountSubsystem;
        this.leftSpeedSup = leftSpeedSup;
        this.rightSpeedSup = rightSpeedSup;

        addRequirements(this.mountSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mountSubsystem.manualMount(
                leftSpeedSup.getAsDouble() * MOUNT_SPEED_SCALAR,
                rightSpeedSup.getAsDouble() * MOUNT_SPEED_SCALAR
        );
    }

    @Override
    public void end(boolean interrupted) {
        mountSubsystem.stopMount();
    }
}
