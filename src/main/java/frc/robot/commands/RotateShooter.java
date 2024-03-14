// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** Rotate the shooter at a constant angular velocity */
public class RotateShooter extends Command {
  private final ShooterSubsystem shooter;
  private final Measure<Velocity<Angle>> angularVelocity;
  private final Rotation2d limit;

  private TrapezoidProfile.Constraints originalConstraints;

  public RotateShooter(ShooterSubsystem shooter, Measure<Velocity<Angle>> angularVelocity, Rotation2d limit) {
    this.shooter = shooter;
    this.angularVelocity = angularVelocity;
    this.limit = limit;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    originalConstraints = shooter.getPitchConstraints();

    shooter.setPitchConstraints(new TrapezoidProfile.Constraints(
      Math.abs(angularVelocity.in(RadiansPerSecond)),
      originalConstraints.maxAcceleration
    ));

    shooter.setPitchGoal(limit, angularVelocity);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setPitchConstraints(originalConstraints);

    if (interrupted) {
      shooter.reset();
    }
  }

  @Override
  public boolean isFinished() {
    return shooter.pitchAtReference();
  }
}
