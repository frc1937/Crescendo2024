package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.RobotState;
import frc.robot.subsystems.Swerve5990;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.ShootingConstants.SPEAKER_TARGET;
import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
import static frc.robot.Constants.Swerve.AZIMUTH_CONTROLLER_TOLERANCE;

/**
 * Should use TeleOpShoot OR make TeleOpShoot use this. No need for both.
 */
@Deprecated
public class AimAtSpeaker extends Command {
    private final Swerve5990 swerve5990;
    private final double scalar;
    private double virtualTargetDistance = 0;

    public AimAtSpeaker(Swerve5990 swerve5990, double scalar) {
        this.swerve5990 = swerve5990;
        this.scalar = scalar;

        addRequirements(swerve5990);
    }

    @Override
    public void initialize() {
        // Predict the position the robot will be in when the NOTE is released
        RobotState predictedState = swerve5990.getHistory().predict(Timer.getFPGATimestamp() + SHOOTING_DELAY);
        Translation2d targetDisplacement = SPEAKER_TARGET.calculateTargetDisplacement(predictedState);
        Translation2d virtualTargetDisplacement = SPEAKER_TARGET.calculateVirtualTargetDisplacement(
                targetDisplacement.getNorm(), targetDisplacement, predictedState.getVelocity());
        virtualTargetDistance = virtualTargetDisplacement.getNorm();
    }

    @Override
    public void execute() {
        RobotState predictedState = swerve5990.getHistory().predict(Timer.getFPGATimestamp() + SHOOTING_DELAY);
        Translation2d targetDisplacement = SPEAKER_TARGET.calculateTargetDisplacement(predictedState);

        Translation2d virtualTargetDisplacement = SPEAKER_TARGET.calculateVirtualTargetDisplacement(virtualTargetDistance, targetDisplacement, predictedState.getVelocity());
        virtualTargetDistance = virtualTargetDisplacement.getNorm();

        swerve5990.driveFieldRelative(0, 0, virtualTargetDisplacement.getAngle().times(scalar));
    }


    @Override
    public boolean isFinished() {
        return swerve5990.azimuthAtGoal(Radians.of(AZIMUTH_CONTROLLER_TOLERANCE));
    }
}