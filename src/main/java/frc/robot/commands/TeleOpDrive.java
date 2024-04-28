package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve5990;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpDrive extends Command {
    private final Swerve5990 swerve5990;
    private final DoubleSupplier
            translationSup,
            strafeSup,
            rotationSup;
    private final BooleanSupplier robotCentricSup;

    public TeleOpDrive(Swerve5990 swerve5990, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.swerve5990 = swerve5990;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        addRequirements(swerve5990);
    }

    @Override
    public void execute() {
        //todo: deadband here.

        swerve5990.drive(translationSup.getAsDouble(), strafeSup.getAsDouble(), rotationSup.getAsDouble(),
                robotCentricSup.getAsBoolean());
    }
}