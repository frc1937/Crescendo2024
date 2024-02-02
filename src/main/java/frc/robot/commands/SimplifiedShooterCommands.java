package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.SimplifiedShooterSubsystem;

public class SimplifiedShooterCommands {
    private final SimplifiedShooterSubsystem simplifiedShooterSubsystem;
    public SimplifiedShooterCommands(SimplifiedShooterSubsystem simplifiedShooterSubsystem) {
        this.simplifiedShooterSubsystem = simplifiedShooterSubsystem;
    }

    public Command startShooter() {
        return new FunctionalCommand(
                () -> {},
                simplifiedShooterSubsystem::startFlywheels,
                interrupted -> simplifiedShooterSubsystem.stopFlywheels(),
                () -> false,
                simplifiedShooterSubsystem
        );
    }

    public Command rotatePivotTest(double pivotPower) {
        return new FunctionalCommand(
                () -> {
                },
                () -> {
                    simplifiedShooterSubsystem.setPivotPower(pivotPower);
                },
                interrupted -> simplifiedShooterSubsystem.stopPivot(),
                () -> false,
                simplifiedShooterSubsystem
        );
    }
}
