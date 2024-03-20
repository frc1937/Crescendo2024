// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignWithAmp extends Command {
  private final DrivetrainSubsystem drivetrain;
  private final Supplier<Translation2d> translationSup;

  public AlignWithAmp(DrivetrainSubsystem drivetrain, Supplier<Translation2d> translationSup) {
    this.drivetrain = drivetrain;
    this.translationSup = translationSup;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.resetAzimuthController();
  }

  @Override
  public void execute() {
    Translation2d translation = translationSup.get().times(Constants.Swerve.MAX_SPEED);

    drivetrain.driveWithAzimuth(
        translation,
        Rotation2d.fromDegrees(90),
        true
    );
  }
}
