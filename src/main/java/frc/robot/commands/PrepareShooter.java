// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PrepareShooter extends Command {
    private final ShooterSubsystem shooter;
    private final ShooterSubsystem.Reference reference;

    public PrepareShooter(ShooterSubsystem shooter, ShooterSubsystem.Reference reference) {
        this.shooter = shooter;
        this.reference = reference;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setReference(reference);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooter.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.atReference();
    }
}
