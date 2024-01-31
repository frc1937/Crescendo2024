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

    public Command shoot(double pivotAngle) {
        return startShooter(pivotAngle)
                .andThen(new WaitCommand(0.5))
                .andThen(() -> shooterSubsystem.setKickerVoltage(4))
                .andThen(stopShooter());
    } //To be used when every mechanism is working

    public Command startShooter(double pivotAngle) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    shooterSubsystem.setPivotSetpoint(Rotation2d.fromDegrees(pivotAngle));
                    shooterSubsystem.startFlywheels();
                },
                interrupted -> stopShooter(),
                () -> false,
                shooterSubsystem
        );
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
}
