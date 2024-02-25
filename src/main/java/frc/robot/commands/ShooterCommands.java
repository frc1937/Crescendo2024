package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShootingStates;

import static frc.robot.Constants.ShootingConstants.FLYWHEEL_MAX_RPM;

public class ShooterCommands {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ShooterCommands(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command accelerateFlywheel(ShootingStates state) {
        return new FunctionalCommand(
                () -> initializeShooterByState(state),
                () -> {},
                interrupted -> shooterSubsystem.setKickerSpeed(0.9),
                () -> false,

                shooterSubsystem
        );
    }

    public Command shootNote(ShootingStates state) {
        return new FunctionalCommand(
                () -> initializeShooterByState(state),
                () -> {
                    SmartDashboard.putBoolean("isFlywheelReady", shooterSubsystem.areFlywheelsReady());
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

    public SequentialCommandGroup receiveFromFeeder() {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.setFlywheelSpeed(-0.55 * FLYWHEEL_MAX_RPM, false);
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(48.65));
                    shooterSubsystem.setKickerSpeed(-0.2);
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
        ).andThen(setKickerSpeed(-0.8).withTimeout(0.7));
    }

    public FunctionalCommand intakeGet() {
        return new FunctionalCommand(
                () -> shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0.5)),
                () -> {
                    if (shooterSubsystem.hasPivotArrived()) {
                        intakeSubsystem.setSpeedPercentage(0.7);
                        shooterSubsystem.setFlywheelSpeed(-3000, false);
                        shooterSubsystem.setKickerSpeed(-0.8);
                    }
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

    public SequentialCommandGroup stopShooter() {
        return new WaitCommand(0.7)
                .andThen(new InstantCommand(() -> {
                    shooterSubsystem.stopKicker();
                    shooterSubsystem.stopFlywheels();
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0));
                }));
    }

    private void initializeShooterByState(ShootingStates state) {
        if (shooterSubsystem.doesSeeNote()) {
            shooterSubsystem.setKickerSpeed(-0.3);
            shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(state.getAngle()));
            shooterSubsystem.setFlywheelSpeed(state.getRpmProportion() * state.getSpeedPercentage() * FLYWHEEL_MAX_RPM, true);
        }
    }
}
