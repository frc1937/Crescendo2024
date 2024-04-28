// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve5990;

import java.util.function.DoubleSupplier;

public class AlignWithAmp extends Command {
    private final Swerve5990 swerve5990;
    private final DoubleSupplier translationSup,
            strafeSup;

    public AlignWithAmp(Swerve5990 swerve5990, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        this.swerve5990 = swerve5990;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;

        addRequirements(swerve5990);
    }

    @Override
    public void initialize() {
        swerve5990.setupAzimuthController();
    }

    @Override
    public void execute() {
        //todo: deadbnad
        swerve5990.driveFieldRelative(
                translationSup.getAsDouble(),
                strafeSup.getAsDouble(),
                Rotation2d.fromDegrees(90)
        );
    }
}
