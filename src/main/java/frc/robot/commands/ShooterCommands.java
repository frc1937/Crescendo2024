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

import java.util.concurrent.atomic.AtomicInteger;

import static frc.robot.Constants.ShootingConstants.FLYWHEEL_MAX_RPM;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_BACKWARDS;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_FORWARD;

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
                () -> {
                },
                interrupted -> shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD),
                () -> false,

                shooterSubsystem
        );
    }

    public Command shootToAmp(ShootingStates state) {
        AtomicInteger i = new AtomicInteger();
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(state.getAngle()));
                    shooterSubsystem.setFlywheelSpeed(500, false);
                },
                () -> {
                    i.getAndIncrement();

                    if (i.get() > 50) {
                        shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD);
                    }
                },
                interrupted -> {
                    shooterSubsystem.stopFlywheels();
                    shooterSubsystem.stopKicker();
                    i.set(0);
                },
                () -> false,

                shooterSubsystem
        );
    }

    public Command shootNote(ShootingStates state) {
        return new FunctionalCommand(
                () -> initializeShooterByState(state),
                () -> {
                    SmartDashboard.putBoolean("isFlywheelReady", shooterSubsystem.areFlywheelsReady());
                    if (shooterSubsystem.doesSeeNoteNoiseless() && shooterSubsystem.areFlywheelsReady() && shooterSubsystem.hasPivotArrived()) {
                        shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD);
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
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(51));
                    shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
                },
                () -> {
                },
                interrupted -> {
                    shooterSubsystem.stopFlywheels();
                    shooterSubsystem.stopKicker();
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0));
                },
                shooterSubsystem::doesSeeNoteNoiseless,
                shooterSubsystem
        ).andThen(setKickerSpeed(KICKER_SPEED_BACKWARDS).withTimeout(0.7));
    }

    public SequentialCommandGroup intakeGet(double kickerDuration) {
        return new FunctionalCommand(
                () -> shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0.5)),
                () -> {
                    if (shooterSubsystem.hasPivotArrived()) {
                        intakeSubsystem.setSpeedPercentage(0.7);
                        shooterSubsystem.setFlywheelSpeed(-3000, false);
                        shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
                    }
                },
                interrupted -> {
                    shooterSubsystem.stopFlywheels();
                    intakeSubsystem.stopMotor();
                    shooterSubsystem.stopKicker();

                },

                shooterSubsystem::doesSeeNoteNoiseless,

                shooterSubsystem
        ).andThen(setKickerSpeed(KICKER_SPEED_BACKWARDS).withTimeout(kickerDuration));
    }

    public Command intakeGet() {
        return intakeGet(0.7);
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

    public Command setAngle(double angle) {
        return new FunctionalCommand(
                () -> shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(angle)),
                () -> {
                },
                interrupt -> shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(angle)),
                () -> false,
                shooterSubsystem
        );
    }

    private void initializeShooterByState(ShootingStates state) {
        if (shooterSubsystem.doesSeeNoteNoiseless()) {
            shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
            shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(state.getAngle()));
            shooterSubsystem.setFlywheelSpeed(state.getRpmProportion() * state.getSpeedPercentage() * FLYWHEEL_MAX_RPM, true);
        }
    }
}
