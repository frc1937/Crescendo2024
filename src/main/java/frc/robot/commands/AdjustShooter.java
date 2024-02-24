// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShootingConstants;

public class AdjustShooter extends Command {
    private final ShooterSubsystem shooter;
    private final Rotation2d pitch;
    private final double velocity;

    /**
     * Creates a new AdjustShooter.
     *
     * @param distance the distance, in metres, from the centre of the robot to the target, all
     *                 projected onto the field. Negative slope will cause the shooter to adjust
     *                 backwards.
     */
    public AdjustShooter(ShooterSubsystem shooter, double slope) {
        this.shooter = shooter;

        // FIXME Move 220 to Constants.java
        if (slope > 0) {
          this.pitch = ShootingConstants.SLOPE_TO_PITCH_MAP.get(slope);
        } else {
          this.pitch = Rotation2d.fromDegrees(220).minus(ShootingConstants.SLOPE_TO_PITCH_MAP.get(slope));
        }
        this.velocity = ShootingConstants.SLOPE_TO_VELOCITY_MAP.get(slope);

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPivotAngle(pitch);
        shooter.setFlywheelSpeed(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooter.stopFlywheels();
        }
    }

    @Override
    public boolean isFinished() {
        return shooter.areFlywheelsReady() && shooter.hasPivotArrived();
    }
}
