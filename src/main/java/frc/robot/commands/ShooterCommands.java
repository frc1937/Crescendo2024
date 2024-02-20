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

    public Command receiveFromFeeder() {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.setFlywheelSpeed(-0.55);
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(49));
                    shooterSubsystem.setKickerSpeed(-0.5);
                },

                () -> {
                },

                interrupted -> {
                    shooterSubsystem.stopFlywheels();
                    shooterSubsystem.stopKicker();
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0));
                },

                shooterSubsystem::doesSeeNote,

                shooterSubsystem
        );
    }

    public Command shootNote(double angle, double speed) {
        return new FunctionalCommand(
                () -> {
                    if (shooterSubsystem.doesSeeNote()) {
                        shooterSubsystem.stopKicker();
                        shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(angle));
                        shooterSubsystem.setFlywheelSpeed(speed);
                    }
                },

                () -> {
                    if (shooterSubsystem.doesSeeNote() && shooterSubsystem.areFlywheelsReady(speed * 5600) && shooterSubsystem.hasPivotArrived()) {
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

    public FunctionalCommand accelerateFlywheel() {
        return new FunctionalCommand(
            () -> {
                    shooterSubsystem.setFlywheelSpeed(0.8);
                },

                () -> {
                },

                interrupted -> {
                    shooterSubsystem.stopFlywheels();
                },

                () -> false,

                shooterSubsystem
        );
    }

    public FunctionalCommand intakeGet() {
        return new FunctionalCommand(
                /* Initialize*/() -> {
                      //intakeStart();
                      shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(-0.7));
                },
                /* Execute */() -> {
                    if(shooterSubsystem.hasPivotArrived()) {
                        intakeSubsystem.setSpeedPercentage(0.9);
                        shooterSubsystem.setFlywheelSpeed(-1);
                        shooterSubsystem.setKickerSpeed(-0.8);
                    }
                },
                /* When done do:*/interrupted -> intakeStop(),
                /* Condition to be done at */ shooterSubsystem::doesSeeNote,

                shooterSubsystem
        );
    }

    private void intakeStart() {
        shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0));

        intakeSubsystem.setSpeedPercentage(0.9);
        shooterSubsystem.setFlywheelSpeed(-1);
        shooterSubsystem.setKickerSpeed(-0.8);
    }

    public void intakeStop() {
        shooterSubsystem.stopFlywheels();
        intakeSubsystem.stopMotor();
        shooterSubsystem.stopKicker();
    }

    public FunctionalCommand stopIntake() {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.stopFlywheels();
                    intakeSubsystem.stopMotor();
                    shooterSubsystem.stopKicker();
                },

                () -> {
                },
                (interrupted) -> {
                },
                () -> false,

                intakeSubsystem
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
                },

                () -> {
                },

                interrupted -> {
                    shooterSubsystem.stopFlywheels();
                    intakeSubsystem.stopMotor();
                    shooterSubsystem.stopKicker();
                },

                () -> !shooterSubsystem.doesSeeNote(),

                shooterSubsystem
        );
    }

    public Command setAngle(double angle) {
        return new FunctionalCommand(
                () -> shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(angle)),
                () -> {
                },
                interrupted -> {
                },//shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0)),
                () -> false,
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
