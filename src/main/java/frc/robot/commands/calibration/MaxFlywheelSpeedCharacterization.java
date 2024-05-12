package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.RPM;

public class MaxFlywheelSpeedCharacterization extends Command {
    private final ShooterSubsystem shooterSubsystem;

    private double maxSpeed = 0;

    public MaxFlywheelSpeedCharacterization(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setFlywheelsSpeed(RPM.of(7000));
    }

    @Override
    public void execute() {
        double currentVelocity = shooterSubsystem.getFlywheelsSpeed();

        if(currentVelocity > maxSpeed) {
            maxSpeed = currentVelocity;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheels();

        System.out.println("Max Speed: " + maxSpeed);
    }
}
