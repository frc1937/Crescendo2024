package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShootingStates;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.*;

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
                    if (shooterSubsystem.isLoaded() && shooterSubsystem.areFlywheelsReady() && shooterSubsystem.isPitchReady()) {
                        shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD);
                    }
                },
                interrupted -> {
                    shooterSubsystem.stopKicker();
                    shooterSubsystem.stopFlywheels();
                    shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(PITCH_INTAKE_FLOOR_ANGLE));
                },
                () -> false,

                shooterSubsystem
        );
    }

    public SequentialCommandGroup receiveFromFeeder() {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.setFlywheelsSpeed(RPM.of(-0.55 * FLYWHEEL_MAX_RPM));
                    shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(PITCH_INTAKE_FEEDER_ANGLE));
                    shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
                },
                () -> {
                },
                interrupted -> {
                    shooterSubsystem.stopFlywheels();
                    shooterSubsystem.stopKicker();
                    shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(PITCH_DEFAULT_ANGLE));
                },
                shooterSubsystem::isLoaded,
                shooterSubsystem
        ).andThen(setKickerSpeed(KICKER_SPEED_BACKWARDS).withTimeout(0.7));
    }

    public Command postIntake() {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                },
                interrupted -> {
                    shooterSubsystem.stopKicker();
                    shooterSubsystem.stopFlywheels();
                    shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(PITCH_DEFAULT_ANGLE));
                },
                () -> false,
                shooterSubsystem)
                .withTimeout(0.7);
    }

    public Command intakeGet(boolean includePostIntake) {
        FunctionalCommand preparePivot = new FunctionalCommand(
                () -> shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(PITCH_INTAKE_FLOOR_ANGLE)),
                () -> {
                },
                interrupt -> {
                },
                shooterSubsystem::isLoaded,
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
                    intakeSubsystem.stopMotor();

                    if (Boolean.TRUE.equals(interrupted)) {
                        shooterSubsystem.stopKicker();
                        shooterSubsystem.stopFlywheels();
                    }
                },
                shooterSubsystem::isLoaded,
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

    public ParallelCommandGroup shootToAmp(ShootingStates state) {
        return new ParallelCommandGroup(
                shootNote(state).andThen(new InstantCommand(() -> shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD)).withTimeout(0.7))
        );
    }

    private void initializeShooterByState(ShootingStates state) {
        shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
        shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(state.getAngle()));
        shooterSubsystem.setFlywheelsSpeed(RPM.of(state.getSpeedPercentage() * shooterSubsystem.theoreticalMaximumVelocity.in(RPM)));
    }
}
