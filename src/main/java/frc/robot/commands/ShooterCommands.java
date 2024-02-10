package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommands {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ShooterCommands(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command shootNote(double angle) {
        return new FunctionalCommand(
                () -> {
                    if(shooterSubsystem.doesSeeNote()) {
                        shooterSubsystem.stopKicker();
                        shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(angle));
                        shooterSubsystem.setFlywheelSpeed(0.9);
                    }
                },

                () -> {
                    if (shooterSubsystem.doesSeeNote() && shooterSubsystem.areFlywheelsReady() && shooterSubsystem.hasPivotArrived()) {
                        shooterSubsystem.setKickerSpeed(0.9);
                    }
                },

                interrupted -> {
                    shooterSubsystem.stopKicker();
                    shooterSubsystem.stopFlywheels();
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0));
                },

                () -> false,

                shooterSubsystem
        );

    }

    public FunctionalCommand floorIntake() {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0));

                    intakeSubsystem.setSpeedPercentage(0.9);
                    shooterSubsystem.setFlywheelSpeed(-0.7);
                    shooterSubsystem.setKickerSpeed(-0.8);
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(1.5));
//todo faster pitch
                },
                () -> {
                },
                interrupted -> {
                    shooterSubsystem.stopFlywheels();
                    intakeSubsystem.stopMotor();
                    shooterSubsystem.stopKicker();
                },
                shooterSubsystem::doesSeeNote,
                shooterSubsystem
        );
    }

    public Command setKickerSpeed(double speed) {
        return new FunctionalCommand(
                () -> shooterSubsystem.setKickerSpeed(speed),
                () -> {
                },
                interrupted -> shooterSubsystem.stopKicker(),
                () -> false,
                shooterSubsystem
        );
    }
}
