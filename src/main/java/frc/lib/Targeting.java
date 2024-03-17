// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import static frc.robot.Constants.ShootingConstants.DISTANCE_TO_TIME_OF_FLIGHT_MAP;
import static frc.robot.Constants.ShootingConstants.FIELD_LENGTH;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

public class Targeting {
    /** Calculate the displacement from the centre of the robot to the target */
    public static Translation2d calculateTargetDisplacement(RobotState robotState, Translation2d blueTargetPosition) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d targetPosition = alliance == DriverStation.Alliance.Red
                                        ? new Translation2d(FIELD_LENGTH, 0).minus(blueTargetPosition)
                                        : blueTargetPosition;
        return targetPosition.minus(robotState.getPose().getTranslation());
    }

    /**
     * Find the displacement to the virtual target, i.e., the target to which the robot should aim s.t.
     * the NOTE enters the actual target, even whilst moving.
     */
    public static Translation2d calculateVirtualTargetDisplacement(double virtualTargetDistance,
                                                                    Translation2d targetDisplacement,
                                                                    ChassisSpeeds velocity) {
        double timeOfFlight = DISTANCE_TO_TIME_OF_FLIGHT_MAP.get(virtualTargetDistance);

        Translation2d displacementDueToRobotVelocity = new Translation2d(
                velocity.vxMetersPerSecond * timeOfFlight,
                velocity.vyMetersPerSecond * timeOfFlight
        );

        return targetDisplacement.minus(displacementDueToRobotVelocity);
    }
}
