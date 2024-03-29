package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.RobotState;
import frc.robot.subsystems.DrivetrainSubsystem;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.ShootingConstants.SPEAKER_TARGET;
import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
import static frc.robot.Constants.Swerve.AZIMUTH_CONTROLLER_TOLERANCE;

/**
 * Should use TeleOpShoot OR make TeleOpShoot use this. No need for both.
 */
@Deprecated
public class AimAtSpeaker extends Command {
    private final DrivetrainSubsystem drivetrain;
    private final double scalar;
    private double virtualTargetDistance = 0;

    public AimAtSpeaker(DrivetrainSubsystem drivetrain, double scalar) {
        this.drivetrain = drivetrain;
        this.scalar = scalar;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Predict the position the robot will be in when the NOTE is released
        RobotState predictedState = drivetrain.getHistory().predict(Timer.getFPGATimestamp() + SHOOTING_DELAY);
        Translation2d targetDisplacement = SPEAKER_TARGET.calculateTargetDisplacement(predictedState);
        Translation2d virtualTargetDisplacement = SPEAKER_TARGET.calculateVirtualTargetDisplacement(
                targetDisplacement.getNorm(), targetDisplacement, predictedState.getVelocity());
        virtualTargetDistance = virtualTargetDisplacement.getNorm();
    }

    @Override
    public void execute() {
        RobotState predictedState = drivetrain.getHistory().predict(Timer.getFPGATimestamp() + SHOOTING_DELAY);
        Translation2d targetDisplacement = SPEAKER_TARGET.calculateTargetDisplacement(predictedState);

        Translation2d virtualTargetDisplacement = SPEAKER_TARGET.calculateVirtualTargetDisplacement(virtualTargetDistance, targetDisplacement, predictedState.getVelocity());
        virtualTargetDistance = virtualTargetDisplacement.getNorm();

        drivetrain.driveWithAzimuth(new Translation2d(0, 0),
                virtualTargetDisplacement.getAngle().times(scalar));
    }


    @Override
    public boolean isFinished() {
        return drivetrain.azimuthAtGoal(Radians.of(AZIMUTH_CONTROLLER_TOLERANCE));
    }
}