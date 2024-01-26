package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
    private final Shooter shooter;
    public ShooterCommands(Shooter shooter) {
        this.shooter = shooter;
    }

    public Command shoot(double pivotAngle, double flywheelAngle) {
        return startShooter(pivotAngle, flywheelAngle)
                .andThen(new WaitCommand(0.5))
                .andThen(() -> shooter.setKickerVoltage(4))
                .andThen(stopShooter());
    }

    private Command stopShooter() {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    shooter.stopFlywheel();
                    shooter.stopKicker();
                },
                interrupted -> {
                },
                () -> false,
                shooter
        );
    }

    private Command startShooter(double pivotAngle, double flywheelAngle) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    shooter.rotatePivot(Rotation2d.fromDegrees(pivotAngle));
                    shooter.rotateFlywheel(Rotation2d.fromDegrees(flywheelAngle));
                },
                interrupted -> {
                },
                () -> false,
                shooter
        );
    }
}
