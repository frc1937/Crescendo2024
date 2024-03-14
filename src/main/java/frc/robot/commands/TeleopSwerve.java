package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier
            translationSup,
            strafeSup,
            rotationSup;
    private final BooleanSupplier robotCentricSup;

    public TeleopSwerve(DrivetrainSubsystem drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.drivetrain = drivetrain;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationValue = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeValue = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
        double rotationValue = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND);

        /* Drive */
        drivetrain.drive(
                new Translation2d(translationValue, strafeValue).times(Constants.Swerve.MAX_SPEED),
                rotationValue * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                !robotCentricSup.getAsBoolean(),
                false);
    }
}