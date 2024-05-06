package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve5990;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleOpDrive extends Command {
    private final Swerve5990 swerve5990;
    private final DoubleSupplier
            translationSupplier,
            strafeSupplier,
            rotationSup;
    private final BooleanSupplier robotCentricSup;

    public TeleOpDrive(Swerve5990 swerve5990, DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.swerve5990 = swerve5990;
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;

        addRequirements(swerve5990);
    }

    @Override
    public void execute() {
        double deadbandTranslation = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.STICK_DEADBAND);
        double deadbandStrafe = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.STICK_DEADBAND);

        swerve5990.drive(deadbandTranslation, deadbandStrafe, rotationSup.getAsDouble(), robotCentricSup.getAsBoolean());
    }
}