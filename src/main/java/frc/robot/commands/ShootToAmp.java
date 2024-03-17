// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_FORWARD;

public class ShootToAmp extends SequentialCommandGroup {
    /**
     * Creates a new ShootToAmp.
     */
    public ShootToAmp(ShooterSubsystem shooter, DrivetrainSubsystem drivetrainSubsystem) {
        Command prepare = new ParallelCommandGroup(
                new TeleOpDriveRetarded(drivetrainSubsystem, () -> 0.1).withTimeout(
                        0.21
                ),

                new SequentialCommandGroup(
                        new WaitCommand(0.4),
                        new PrepareShooter(shooter, new ShooterSubsystem.Reference(Rotation2d.fromDegrees(
                                104
                        ), RPM.of(600)))
                        .withTimeout(2)
                )
        );

        Command release = shooter.startEnd(
                () -> {
                    shooter.setKickerSpeed(KICKER_SPEED_FORWARD);
                    shooter.setFlywheelsSpeed(RPM.of(600));
                },
                shooter::reset
        ).withTimeout(1);

        addCommands(
                prepare,
                release
        );
    }
}
