package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpDrive extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier
            translationSup,
            strafeSup,
            rotationSup;
    private final BooleanSupplier robotCentricSup;

    public TeleOpDrive(DrivetrainSubsystem drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.drivetrain = drivetrain;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Translation2d stickTranslation = new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble());
        
        // Deadband the joysticks
        if (stickTranslation.getNorm() < Constants.STICK_DEADBAND) {
            stickTranslation = new Translation2d();
        }
        double rotationValue = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND);

        Translation2d translation = stickTranslation.times(Constants.Swerve.MAX_SPEED);

        drivetrain.drive(
                translation,
                rotationValue * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !robotCentricSup.getAsBoolean(),
                true);
    }
}