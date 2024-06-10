package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve5990;

public class TeleOpRotate extends Command {
    private final Swerve5990 swerve5990;
    private final Rotation2d targetRotation;

    public TeleOpRotate(Swerve5990 swerve5990, Rotation2d targetRotation) {
        this.swerve5990 = swerve5990;
        this.targetRotation = targetRotation;

        addRequirements(swerve5990);
    }

    @Override
    public void execute() {
        swerve5990.driveFieldRelative(0, 0, targetRotation);
//                deadbandTranslation, deadbandStrafe, targetRotation.getAsDouble(), robotCentricSup.getAsBoolean());
    }
}