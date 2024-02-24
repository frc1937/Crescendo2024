package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier
            translationSup,
            strafeSup,
            rotationSup;

    public TeleopSwerve(SwerveSubsystem swerveSubsystem, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.swerveSubsystem = swerveSubsystem;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationValue = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeValue = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
        double rotationValue = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND);

        /* Drive */
        swerveSubsystem.drive(
                new Translation2d(translationValue, strafeValue).times(Constants.Swerve.MAX_SPEED),
                rotationValue * Constants.Swerve.MAX_ANGULAR_VELOCITY
        );
    }
}