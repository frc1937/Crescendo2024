package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.RobotState;
import frc.lib.Targeting;
import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.ShootingConstants.BLUE_TARGET_POSITION;
import static frc.robot.Constants.ShootingConstants.DISTANCE_TO_TIME_OF_FLIGHT_MAP;
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
        Translation2d targetDisplacement = Targeting.calculateTargetDisplacement(predictedState);
        Translation2d virtualTargetDisplacement = Targeting.calculateVirtualTargetDisplacement(
                targetDisplacement.getNorm(), targetDisplacement, predictedState.getVelocity());
        virtualTargetDistance = virtualTargetDisplacement.getNorm();
    }

    @Override
    public void execute() {
        // Get values, deadband
        // Predict the position the robot will be in when the NOTE is released
        RobotState predictedState = RobotState.predict(drivetrain.getPoseHistory(), Timer.getFPGATimestamp() + SHOOTING_DELAY);
        Translation2d targetDisplacement = Targeting.calculateTargetDisplacement(predictedState);

        // Calculate the displacement of the virtual target, to which the robot so it can
        // score whilst moving
        Translation2d virtualTargetDisplacement = Targeting.calculateVirtualTargetDisplacement(virtualTargetDistance, targetDisplacement, predictedState.getVelocity());
        virtualTargetDistance = virtualTargetDisplacement.getNorm();

        // Aim the azimuth to the virtual target
        drivetrain.driveWithAzimuth(new Translation2d(0, 0),
                virtualTargetDisplacement.getAngle().times(scalar));
    }


    @Override
    public boolean isFinished() {
        return drivetrain.azimuthAtGoal();
    }
}