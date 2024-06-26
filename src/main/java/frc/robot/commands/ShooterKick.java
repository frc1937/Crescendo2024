// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static frc.robot.subsystems.shooter.ShooterConstants.KICKER_SPEED_FORWARD;


public class ShooterKick extends Command {
    private final ShooterSubsystem shooter;

    public ShooterKick(ShooterSubsystem shooter) {
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setKickerSpeed(KICKER_SPEED_FORWARD);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.reset();
    }
}
