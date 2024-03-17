package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

@Deprecated
public class TeleOpDriveRetarded extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier
            translationSup;

    public TeleOpDriveRetarded(DrivetrainSubsystem drivetrain, DoubleSupplier translationSup) {
        this.drivetrain = drivetrain;
        this.translationSup = translationSup;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Translation2d stickTranslation = new Translation2d(translationSup.getAsDouble(), 0);

        // Deadband the joysticks
        if (stickTranslation.getNorm() < Constants.STICK_DEADBAND) {
            stickTranslation = new Translation2d();
        }

        Translation2d translation = stickTranslation.times(Constants.Swerve.MAX_SPEED);

        drivetrain.drive(
                translation,
                0,
                false,
                true);
    }

    @Override
    public void end(boolean interrupt) {
        drivetrain.drive(new Translation2d(0, 0), 0, true, true);
    }
}