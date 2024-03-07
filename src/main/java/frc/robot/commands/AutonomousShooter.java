package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.RPM;

public class AutonomousShooter {
    private final ShooterSubsystem shooterSubsystem;

    public AutonomousShooter(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    public Command adjustShooter(double degrees, int rpm) {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.setPivotAngle(Rotation2d.fromDegrees(degrees));
                    shooterSubsystem.setFlywheelsSpeed(RPM.of(rpm));
                },
                () -> {
                },
                interrupted -> {},
                () -> {
                    boolean isReady = shooterSubsystem.areFlywheelsReady() && shooterSubsystem.hasPivotArrived();
                    return isReady || (!shooterSubsystem.doesSeeNoteNoiseless());
                },

                shooterSubsystem);
    }
}
