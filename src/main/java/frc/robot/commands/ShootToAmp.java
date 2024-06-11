// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve5990;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.shooter.ShooterConstants.AMP_INIT;
import static frc.robot.subsystems.shooter.ShooterConstants.KICKER_SPEED_FORWARD;

public class ShootToAmp extends SequentialCommandGroup {
    /**
     * Creates a new ShootToAmp.
     */
    public ShootToAmp(ShooterSubsystem shooter, Swerve5990 swerve5990, LEDsSubsystem leds) {
        Command prepare = new ParallelCommandGroup(
                new TeleOpDrive(swerve5990, () -> 0.3, () -> 0, () -> 0, () -> true).withTimeout(0.2),
                new PrepareShooter(shooter, AMP_INIT).withTimeout(1.9 + 0.37)
        );

        Command release = new ParallelDeadlineGroup(
            shooter.startEnd(
                    () -> {
                        shooter.setKickerSpeed(KICKER_SPEED_FORWARD);
                        shooter.setFlywheelsSpeed(RPM.of(500));
                    },
                    shooter::reset
            )
        ).withTimeout(1.5);

        addCommands(
                prepare,
                release
        );
    }
}
