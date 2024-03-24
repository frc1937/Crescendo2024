// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ShootingConstants.FIELD_LENGTH;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AlignWithChain extends Command {
  private final DrivetrainSubsystem drivetrain;
  private final DoubleSupplier translationSup,
                               strafeSup;

  public AlignWithChain(DrivetrainSubsystem drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
    this.drivetrain = drivetrain;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.resetAzimuthController();
  }

  @Override
  public void execute() {
    Translation2d stickTranslation = new Translation2d(translationSup.getAsDouble(), strafeSup.getAsDouble());

    if (stickTranslation.getNorm() < Constants.STICK_DEADBAND) {
      stickTranslation = new Translation2d();
    }

    Translation2d translation = stickTranslation.times(Constants.Swerve.MAX_SPEED);

    Translation2d centre = new Translation2d(4.87, 4.11);

    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
      centre = new Translation2d(FIELD_LENGTH - centre.getX(), centre.getY());
    }

    int angle;
    
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
      angle = (int)(centre.minus(drivetrain.getPose().getTranslation()).getAngle().getRotations() * 3.d + 0.5);
      angle = 120 * angle;
    } else {
      angle = (int)(centre.minus(drivetrain.getPose().getTranslation()).getAngle().getRotations() * 3.d);
      angle = 120 * angle + 60;
    }

    drivetrain.driveWithAzimuth(
        translation,
        Rotation2d.fromDegrees(angle),
        true
    );
  }
}
