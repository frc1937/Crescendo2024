// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.calibration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve5990;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AzimuthDrive extends Command {
    private final Swerve5990 swerve5990;
    private final DoubleSupplier translationSup, strafeSup, rotationSup;
    private final BooleanSupplier overrideAzimuthSup;

    private boolean lastOverrideAzimuth;

    public AzimuthDrive(Swerve5990 swerve5990, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier overrideAzimuthSup) {
        this.swerve5990 = swerve5990;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.overrideAzimuthSup = overrideAzimuthSup;

        addRequirements(swerve5990);
    }

    @Override
    public void initialize() {
        swerve5990.publishControllerGains();
    }

    @Override
    public void execute() {
        double translationValue = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeValue = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
        double rotationValue = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND);
        boolean overrideAzimuth = overrideAzimuthSup.getAsBoolean();

        if (overrideAzimuth) {
            if (!lastOverrideAzimuth) {
                swerve5990.rereadControllerGains();
            }

            swerve5990.driveFieldRelative(translationValue, strafeValue, Rotation2d.fromDegrees(90));
        } else {
            swerve5990.drive(
                    translationValue, strafeValue,
                    rotationValue,
                    true);
        }

        lastOverrideAzimuth = overrideAzimuth;
    }
}
