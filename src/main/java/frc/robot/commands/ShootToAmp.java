// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_FORWARD;

public class ShootToAmp extends SequentialCommandGroup {
    public ShootToAmp(ShooterSubsystem shooter, DrivetrainSubsystem drivetrain) {
        // First, move the shooter to the initial position and accelerate the flywheels
        FunctionalCommand prepareShooter = new FunctionalCommand(
                () -> shooter.setReference(new ShooterSubsystem.Reference(Rotation2d.fromDegrees(80), RPM.of(600))),
                () -> {
                },
                interrupted -> {
                    if (interrupted) {
                        shooter.reset();
                    }
                },
                shooter::pitchAtReference,
                shooter
        );

        // Then, start rotating the pitch backwards whilst driving forward
        ParallelDeadlineGroup shoot = new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        // Rotate the pitch backwards slowly
                        new RotateShooter(shooter, DegreesPerSecond.of(10), Rotation2d.fromDegrees(90)),

                        // Then release the note
                        new InstantCommand(() -> shooter.setKickerSpeed(KICKER_SPEED_FORWARD), shooter),

                        // Then continue rotating without stopping
                        new RotateShooter(shooter, DegreesPerSecond.of(10), Rotation2d.fromDegrees(100)),

                        // Then reset the shooter
                        new InstantCommand(shooter::reset, shooter)
                ),

                // Meanwhile, drive forward, away from the amp
                drivetrain.startEnd(
                        () -> drivetrain.drive(new ChassisSpeeds(MetersPerSecond.of(0.5),
                                MetersPerSecond.of(0),
                                DegreesPerSecond.of(0))),
                        drivetrain::stop
                )
        );

        addCommands(prepareShooter, shoot);
        addRequirements(shooter);
    }
}
