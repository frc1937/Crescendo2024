package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TeleOpDrive extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final Supplier<Translation2d> translationSup;
    private final DoubleSupplier rotationSup;
    private final BooleanSupplier robotCentricSup;

    public TeleOpDrive(DrivetrainSubsystem drivetrain, Supplier<Translation2d> translationSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.drivetrain = drivetrain;
        this.translationSup = translationSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Translation2d translation = translationSup.get().times(Constants.Swerve.MAX_SPEED);

        drivetrain.drive(
                translation,
                rotationSup.getAsDouble() * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}