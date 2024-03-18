package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class RotateToAmp extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier
            translationSup,
            strafeSup;

    public RotateToAmp(DrivetrainSubsystem drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        this.drivetrain = drivetrain;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Translation2d stickTranslation = new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble());

        if (stickTranslation.getNorm() < Constants.STICK_DEADBAND) {
            stickTranslation = new Translation2d();
        }

        Translation2d translation = stickTranslation.times(Constants.Swerve.MAX_SPEED);

        double angle = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? -90 : 90;

        drivetrain.driveWithAzimuth(
                translation,
                Rotation2d.fromDegrees(angle),
                true
        );
    }
}
