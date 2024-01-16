package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommands {
    private static final ShooterSubsystem SHOOTER = ShooterSubsystem.getInstance();

    public static Command shoot(double pivotAngle, double flywheelAngle) {
        return new FunctionalCommand(
                () -> {},
                () -> {
                    SHOOTER.rotatePivot(Rotation2d.fromDegrees(pivotAngle));
                    SHOOTER.rotateFlywheel(Rotation2d.fromDegrees(flywheelAngle));
                },
                interrupted -> {},
                () -> false,
                SHOOTER
        ).andThen(new WaitCommand(0.5)).andThen(() -> SHOOTER.enableKicker(4.0));
    }
}
