// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.ShootingConstants.SHOOTER_VERTICAL_ANGLE;

public class AdjustShooter extends Command {
    private final ShooterSubsystem shooter;
    private final Rotation2d pitch;
    private final double velocity;

    /**
     * Creates a new AdjustShooter.
     * @param slope the slope of the shooter
     */
    public AdjustShooter(ShooterSubsystem shooter, double slope) {
        this.shooter = shooter;

        if (slope > 0) {
            pitch = ShootingConstants.SLOPE_TO_PITCH_MAP.get(slope);
        } else {
            slope = -slope;
            pitch = Rotation2d.fromDegrees(SHOOTER_VERTICAL_ANGLE * 2.0).minus(ShootingConstants.SLOPE_TO_PITCH_MAP.get(slope));
        }

        this.velocity = ShootingConstants.SLOPE_TO_VELOCITY_MAP.get(slope);

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPivotAngle(pitch);
        shooter.setFlywheelSpeed(velocity, true);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooter.stopFlywheels();
        }
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBooleanArray("flywheels | pitch", new boolean[]{shooter.areFlywheelsReady(), shooter.hasPivotArrived() });

        boolean isReady = shooter.areFlywheelsReady() && shooter.hasPivotArrived();

        // Avoid adjusting the shooter without a present NOTE. This helps optimising the autonomous
        // when a NOTE wasn't successfully collected.
        return isReady || (!shooter.doesSeeNoteNoiseless());
    }
}
