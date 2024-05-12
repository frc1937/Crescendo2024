package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve5990;

public class MaxDrivetrainSpeedCharacterization extends Command {
    private final Swerve5990 swerve5990;

    private double maxSpeed = 0;

    public MaxDrivetrainSpeedCharacterization(Swerve5990 swerve5990) {
        this.swerve5990 = swerve5990;
    }

    @Override
    public void initialize() {
        //Straighten wheels
        swerve5990.driveSelfRelative(0.02, 0, 0);
    }

    @Override
    public void execute() {
        //todo: check if this is top speed
        swerve5990.drive(1, 0, 0, false);

        double currentXVelocity = swerve5990.getSelfRelativeVelocity().vxMetersPerSecond;

        SmartDashboard.putNumber("/Characterization/Swerve/CurrentVelocity", currentXVelocity);
        SmartDashboard.putNumber("/Characterization/Swerve/MaxVelocity", maxSpeed);

        if(currentXVelocity > maxSpeed) {
            maxSpeed = currentXVelocity;
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("/Characterization/Swerve/MaxVelocity", maxSpeed);
    }
}
