package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.INTAKE;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_BACKWARDS;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_FORWARD;

public class ShooterCommands {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ShooterCommands(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public Command shootNote(ShooterSubsystem.Reference reference) {
        return new FunctionalCommand(
                () -> initializeShooter(false, reference),
                () -> {
                    if (shooterSubsystem.isLoaded() && shooterSubsystem.flywheelsAtReference() && shooterSubsystem.pitchAtReference()) {
                        shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD);
                    }
                },
                interrupted -> shooterSubsystem.reset(),
                () -> false,

                shooterSubsystem
        );
    }

    public Command postIntake() {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.setFlywheelsSpeed(RPM.of(-5000));
                    shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
                },
                () -> {
                },
                interrupted -> shooterSubsystem.reset(),
                () -> false,
                shooterSubsystem).withTimeout(1.2);
    }

    public Command floorIntake(boolean includePostIntake) {
        Command prepareAndOperateIntake = new FunctionalCommand(
                () -> {
                    initializeShooter(true, INTAKE);

                    intakeSubsystem.setSpeedPercentage(0.7);
                },
                () -> {
                }
                ,
                (interrupted) -> {
                    shooterSubsystem.reset();
                    intakeSubsystem.stopMotor();
                },

                shooterSubsystem::isLoaded,
                shooterSubsystem
        );

        if (includePostIntake) {
            return prepareAndOperateIntake.andThen(postIntake());
        } else {
            return prepareAndOperateIntake;
        }
    }

    public Command floorIntake() {
        return floorIntake(true);
    }

    private void initializeShooter(boolean shouldUseKicker, ShooterSubsystem.Reference reference) {
        if(shouldUseKicker)
            shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);

        shooterSubsystem.setPitchGoal(reference.pitchPosition);
        shooterSubsystem.setFlywheelsSpeed(reference.flywheelVelocity);
    }
}
