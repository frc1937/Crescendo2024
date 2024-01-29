package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommands {
    private final ShooterSubsystem shooterSubsystem;
    public ShooterCommands(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    public Command shoot(double pivotAngle, double flywheelAngle) {
        return startShooter(pivotAngle, flywheelAngle)
                .andThen(new WaitCommand(0.5))
                .andThen(() -> shooterSubsystem.setKickerVoltage(4))
                .andThen(stopShooter());
    }

    private Command stopShooter() {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    shooterSubsystem.stopFlywheel();
                    shooterSubsystem.stopKicker();
                },
                interrupted -> {
                },
                () -> false,
                shooterSubsystem
        );
    }

    private Command startShooter(double pivotAngle, double flywheelAngle) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    shooterSubsystem.rotatePivot(Rotation2d.fromDegrees(pivotAngle));
                    shooterSubsystem.rotateFlywheel(Rotation2d.fromDegrees(flywheelAngle));
                },
                interrupted -> {
                },
                () -> false,
                shooterSubsystem
        );
    }
}
