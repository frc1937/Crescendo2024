package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.leds.OsculatingStrip;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_MAX_RPM;
import static frc.robot.Constants.ShootingConstants.INTAKE;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_BACKWARDS;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_FORWARD;
import static frc.robot.Constants.ShootingConstants.PITCH_INTAKE_FEEDER_ANGLE;

public class ShooterCommands {
    private final ShooterSubsystem shooterSubsystem;
    private final LEDsSubsystem leds;
    private final IntakeSubsystem intakeSubsystem;

    public ShooterCommands(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, LEDsSubsystem leds) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.leds = leds;
    }


    public FunctionalCommand receiveFromFeeder() {
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
                },
                shooterSubsystem::isLoaded,
                shooterSubsystem
        );
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
                    shooterSubsystem.setFlywheelsSpeed(RPM.of(-2000));
                    shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
                    intakeSubsystem.setSpeedPercentage(0.4);
                },
                () -> {},
                interrupted -> {
                    shooterSubsystem.reset();
                    intakeSubsystem.stopMotor();
                },
                () -> false,
                shooterSubsystem
        );
    }

    public Command floorIntake() {
        return new ParallelDeadlineGroup(
            new FunctionalCommand(
                () -> {
                    initializeShooter(true, INTAKE);
                    intakeSubsystem.setSpeedPercentage(0.8);
                },
                () -> {},
                interrupted -> {},
                shooterSubsystem::isLoaded,
                shooterSubsystem
            )//,
            //new OsculatingStrip(leds, shooterSubsystem)
        );
    }

    private void initializeShooter(boolean shouldUseKicker, ShooterSubsystem.Reference reference) {
        if(shouldUseKicker)
            shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);

        shooterSubsystem.setReference(reference);
    }
}
