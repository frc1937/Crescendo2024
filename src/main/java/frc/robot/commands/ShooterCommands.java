package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShootingStates;

import static edu.wpi.first.units.Units.RPM;
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

    public Command shootNote(ShootingStates state) {
        return new FunctionalCommand(
                () -> initializeShooterByState(state),
                () -> {
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
                    shooterSubsystem.setFlywheelsSpeed(RPM.of(-0.55 * FLYWHEEL_MAX_RPM));
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

    public Command postIntake() {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                },
                interrupted -> shooterSubsystem.stopKicker(),
                () -> false,

                shooterSubsystem).withTimeout(0.7);
    }

    public Command intakeGet(boolean includePostIntake) {
        FunctionalCommand preparePivot = new FunctionalCommand(
                () -> shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0.5)),
                () -> {
                },
                interrupt -> {
                },
                shooterSubsystem::hasPivotArrived,
                shooterSubsystem);

        FunctionalCommand operateIntake = new FunctionalCommand(
                () -> {
                    intakeSubsystem.setSpeedPercentage(0.7);
                    shooterSubsystem.setFlywheelsSpeed(RPM.of(-3000));
                    shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
                },
                () -> {
                },
                interrupted -> {
                    shooterSubsystem.stopFlywheels();
                    intakeSubsystem.stopMotor();

                    if (Boolean.TRUE.equals(interrupted)) {
                        shooterSubsystem.stopKicker();
                    }
                },
                shooterSubsystem::doesSeeNoteNoiseless,
                intakeSubsystem, shooterSubsystem);

        Command prepareAndOperateIntake = preparePivot.andThen(operateIntake);

        if (includePostIntake) {
            return prepareAndOperateIntake.andThen(postIntake());
        } else {
            return prepareAndOperateIntake;
        }
    }

    public Command intakeGet() {
        return intakeGet(true);
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

    public SequentialCommandGroup shootToAmp(ShootingStates state) {
        return shootNote(state)
                .andThen(new InstantCommand(() -> shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD))
                        .withTimeout(0.7));
    }

    private void initializeShooterByState(ShootingStates state) {
//        if (shooterSubsystem.doesSeeNoteNoiseless()) {
        shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
        shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(state.getAngle()));
        shooterSubsystem.setFlywheelsSpeed(RPM.of(state.getRpmProportion() * state.getSpeedPercentage() * FLYWHEEL_MAX_RPM));
//        }
    }
}
