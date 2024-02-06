package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommands {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ShooterCommands(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public SequentialCommandGroup takeNoteAndPivotToPreset() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0))),
                new InstantCommand(() -> intakeSubsystem.setSpeedPercentage(0.5)),
                new InstantCommand(() -> shooterSubsystem.setFlywheelSpeed(-0.5))
        );
    }

    public SequentialCommandGroup restPivotHighAndShootNote() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(30))),
                new InstantCommand(() -> shooterSubsystem.setFlywheelSpeed(0.8))
        );
    }

    public Command rotateFlywheels(double speed) {
        return new FunctionalCommand(
                () -> {
                },
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
