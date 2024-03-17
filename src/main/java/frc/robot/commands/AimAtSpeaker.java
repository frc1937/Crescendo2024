package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.RobotState;
import frc.lib.Target;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.ShootingConstants.BLUE_SPEAKER_TARGET;
import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;

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
        RobotState predictedState = RobotState.predict(drivetrain.getPoseHistory(), Timer.getFPGATimestamp() + SHOOTING_DELAY);
        Translation2d targetDisplacement = BLUE_SPEAKER_TARGET.calculateTargetDisplacement(predictedState);
        Translation2d virtualTargetDisplacement = Target.calculateVirtualTargetDisplacement(
                targetDisplacement.getNorm(), targetDisplacement, predictedState.getVelocity());
        virtualTargetDistance = virtualTargetDisplacement.getNorm();
    }

    @Override
    public void execute() {
        RobotState predictedState = RobotState.predict(drivetrain.getPoseHistory(), Timer.getFPGATimestamp() + SHOOTING_DELAY);
        Translation2d targetDisplacement = BLUE_SPEAKER_TARGET.calculateTargetDisplacement(predictedState);

        Translation2d virtualTargetDisplacement = Target.calculateVirtualTargetDisplacement(virtualTargetDistance, targetDisplacement, predictedState.getVelocity());
        virtualTargetDistance = virtualTargetDisplacement.getNorm();

        drivetrain.driveWithAzimuth(new Translation2d(0, 0),
                virtualTargetDisplacement.getAngle().times(scalar));
    }


    @Override
    public boolean isFinished() {
        return drivetrain.azimuthAtGoal();
    }
}