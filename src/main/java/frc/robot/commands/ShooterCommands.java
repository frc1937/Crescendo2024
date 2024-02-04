package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommands {
    private final ShooterSubsystem shooterSubsystem;
    public ShooterCommands(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    public Command rotateFlywheels(double speed) {
        return new FunctionalCommand(
                () -> {},
                () -> shooterSubsystem.setFlywheelSpeed(speed),
                interrupted -> shooterSubsystem.stopFlywheels(),
                () -> false,
                shooterSubsystem
        );
    }

    public Command rotatePivot(Rotation2d rotation) {
        return new FunctionalCommand(
                () -> {
                },
                () -> shooterSubsystem.setPivotAngle(rotation),
                interrupted -> shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0)),
                () -> false,
                shooterSubsystem
        );
    }
}
