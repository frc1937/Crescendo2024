// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.calibration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AzimuthDrive extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier translationSup, strafeSup, rotationSup;
    private final BooleanSupplier overrideAzimuthSup;

    private boolean lastOverrideAzimuth;

    public AzimuthDrive(DrivetrainSubsystem drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier overrideAzimuthSup) {
        this.drivetrain = drivetrain;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.overrideAzimuthSup = overrideAzimuthSup;

        addRequirements(drivetrain);

    }

    @Override
    public void initialize() {
        drivetrain.publishControllerGains();
    }

    @Override
    public void execute() {
        double translationValue = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.STICK_DEADBAND);
        double strafeValue = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.STICK_DEADBAND);
        double rotationValue = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.STICK_DEADBAND);
        boolean overrideAzimuth = overrideAzimuthSup.getAsBoolean();

        if (overrideAzimuth) {
            if (!lastOverrideAzimuth) {
                drivetrain.rereadControllerGains();
            }

            drivetrain.driveWithAzimuth(new Translation2d(translationValue, strafeValue).times(Constants.Swerve.MAX_SPEED), Rotation2d.fromDegrees(90));
        } else {
            drivetrain.drive(
                    new Translation2d(translationValue, strafeValue).times(Constants.Swerve.MAX_SPEED),
                    rotationValue * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                    true,
                    true);
        }

        lastOverrideAzimuth = overrideAzimuth;
    }
}
