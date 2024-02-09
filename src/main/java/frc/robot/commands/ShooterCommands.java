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
                    shooterSubsystem.stopKicker();
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(angle));
                    shooterSubsystem.setFlywheelSpeed(0.9);
                },

                () -> {
                    if (shooterSubsystem.areFlywheelsReady() && shooterSubsystem.hasPivotArrived()) {
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
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(2.5));

                    intakeSubsystem.setSpeedPercentage(0.9);
                    shooterSubsystem.setFlywheelSpeed(-0.7);
                    shooterSubsystem.setKickerSpeed(-0.8);
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

//    public SequentialCommandGroup restPivotHighAndShootNote() {
//        return setPivotAngle(Rotation2d.fromDegrees(30))
//                .andThen(setFlywheelSpeed(0.8, true))
//                .andThen(new WaitUntilCommand(shooterSubsystem::areFlywheelsReady))
//                .andThen(setKickerSpeed(0.3));
//
////        return new SequentialCommandGroup(
////                setPivotAngle(Rotation2d.fromDegrees(30)),
////                // new InstantCommand(() -> shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(30))),
////                setFlywheelSpeed(0.8, false),
////                new WaitUntilCommand(shooterSubsystem::areFlywheelsReady),
////                //new InstantCommand(() -> new WaitUntilCommand(shooterSubsystem::areFlywheelsReady)),
////                // todo: So many things could go wrong. Please test.
////                // new InstantCommand(() -> shooterSubsystem.setKickerSpeed(0.3))
////                setKickerSpeed(0.3)
////        );
//    }

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

    public Command setFlywheelSpeed(double speed, boolean toCompare) {
        return new FunctionalCommand(
                () -> shooterSubsystem.setFlywheelSpeed(speed),
                () -> {
                },
                interrupted -> shooterSubsystem.stopFlywheels(),
                () -> shooterSubsystem.doesSeeNote() == toCompare,
                shooterSubsystem
        );
    }

    public Command setPivotAngle(Rotation2d rotation) {
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
