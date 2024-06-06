package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_MAX_RPM;

public class MaxFlywheelSpeedCharacterization extends Command {
    private final ShooterSubsystem shooterSubsystem;

    private double maxFlywheelSpeed = 0;

    public MaxFlywheelSpeedCharacterization(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setFlywheelsSpeed(RPM.of(FLYWHEEL_MAX_RPM));
    }

    @Override
    public void execute() {
        double currentVelocity = shooterSubsystem.getFlywheelsSpeed();

        SmartDashboard.putNumber("Characterization/Flywheel/CurrentVelocity [RPM]", currentVelocity);
        SmartDashboard.putNumber("Characterization/Flywheel/MaxVelocity [RPM]", maxFlywheelSpeed);

        if(currentVelocity > maxFlywheelSpeed) {
            maxFlywheelSpeed = currentVelocity;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheels();
        SmartDashboard.putNumber("/Characterization/Flywheel/MaxVelocity [RPM]", maxFlywheelSpeed);
    }
}
