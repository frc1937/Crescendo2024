// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve5990;

import java.util.function.DoubleSupplier;

public class AlignWithAmp extends Command {
    private final Swerve5990 swerve5990;
    private final DoubleSupplier translationSupplier,
            strafeSupplier;

    public AlignWithAmp(Swerve5990 swerve5990, DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier) {
        this.swerve5990 = swerve5990;
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;

        addRequirements(swerve5990);
    }

    @Override
    public void execute() {
        double deadbandTranslation = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.STICK_DEADBAND);
        double deadbandStrafe = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.STICK_DEADBAND);

        swerve5990.driveFieldRelative(
                deadbandTranslation,
                deadbandStrafe,
                Rotation2d.fromDegrees(90)
        );
    }
}
