package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShootingStates;

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

    public Command shootNote(ShootingStates state) {
        return new FunctionalCommand(
                () -> {
                    if (shooterSubsystem.doesSeeNote()) {
                        shooterSubsystem.stopKicker();
                        shooterSubsystem.setKickerSpeed(-0.3);
                        shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(state.getAngle()));
                        shooterSubsystem.setFlywheelSpeed(state.getSpeedPercentage());
                    }
                },

                () -> {
                    double finalSpeed = state.getRpmProportion() * state.getSpeedPercentage() * 6400;

                    SmartDashboard.putBoolean("isFlywheelReady", shooterSubsystem.areFlywheelsReady(finalSpeed));
                    if (shooterSubsystem.doesSeeNote() && shooterSubsystem.areFlywheelsReady(finalSpeed) && shooterSubsystem.hasPivotArrived()) {
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
            () -> shooterSubsystem.setFlywheelSpeed(0.8),

                () -> {
                },
                interrupted -> shooterSubsystem.stopFlywheels(),

                () -> false,

                shooterSubsystem
        );
    }

    public FunctionalCommand intakeGet() {
        return new FunctionalCommand(
                /* Initialize*/() -> shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(0.5)),
                /* Execute */() -> {
                    if(shooterSubsystem.hasPivotArrived()) {
                        intakeSubsystem.setSpeedPercentage(0.7);
                        shooterSubsystem.setFlywheelSpeed(-1);
                        shooterSubsystem.setKickerSpeed(-0.8);
                    }
                },
                /* When done do:*/interrupted -> {
                                    shooterSubsystem.stopFlywheels();
                                    intakeSubsystem.stopMotor();
                                    shooterSubsystem.stopKicker();
                    },
                /* Condition to be done at */ shooterSubsystem::doesSeeNote,

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
                () -> {},
                interrupted -> shooterSubsystem.stopKicker(),
                () -> false,
                shooterSubsystem
        );
    }
 

    // Shoot into the trap in different speeds and angles
    public Command ShootTrap() {
        return new FunctionalCommand(
                () -> {
                    double angle = shooterSubsystem.getTrapAngle();
                    double speed = shooterSubsystem.getTrapSpeed();

                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(angle));
                    shooterSubsystem.setFlywheelSpeed(speed);
                },
                () -> {
                    double speed = shooterSubsystem.getTrapSpeed();
                    double rpmProportion = shooterSubsystem.getTrapRPM();

                    double finalSpeed = speed * rpmProportion * 6400;

                    if (shooterSubsystem.areFlywheelsReady(finalSpeed) && shooterSubsystem.hasPivotArrived()) {
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


    // Move the pitch towards the trap and shoot at the same time
    public Command PitchShootTrap() {
        
        return new FunctionalCommand(
                () -> {
                    double speed = shooterSubsystem.getTrapSpeed();
                    shooterSubsystem.setFlywheelSpeed(speed);
                    shooterSubsystem.setFlag(false);
                },
                () -> {
                    double angle = shooterSubsystem.getTrapAngle();
                    double speed = shooterSubsystem.getTrapSpeed();
                    double rpmProportion = shooterSubsystem.getTrapRPM();
                    
                    double finalSpeed = speed * rpmProportion * 6400;

                    // The percent of the desired pitch angle to start to shoot at
                    double angleShootVar = 0.7;

                    if (shooterSubsystem.areFlywheelsReady(finalSpeed)) {
                        if(!shooterSubsystem.getFlag()){
                            shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(angle));
                            shooterSubsystem.setFlag(true);
                        }
                    }

                    if(shooterSubsystem.getFlag() && shooterSubsystem.getPivotAngle() >= angle * angleShootVar){
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


}
