// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.DISTANCE_TO_TIME_OF_FLIGHT_MAP;
import static frc.robot.Constants.ShootingConstants.FIELD_LENGTH;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ShooterSubsystem;

public class Target {
    private final Translation2d bluePosition;
    private final InterpolatingTreeMap<Double, ShooterSubsystem.Reference> distanceToReferenceMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShooterSubsystem.Reference::interpolate);

    public Target(Translation2d bluePosition) {
        this.bluePosition = bluePosition;
    }

    /** Calculate the displacement from the centre of the robot to the target */
    public Translation2d calculateTargetDisplacement(RobotState robotState) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d targetPosition = alliance == DriverStation.Alliance.Red
                                        ? new Translation2d(FIELD_LENGTH, 0).minus(bluePosition)
                                        : bluePosition;
        return targetPosition.minus(robotState.getPose().getTranslation());
    }

    /**
     * A small helper function for adding measurements from Constants
     */
    public void putMeasurement(double distance, double degrees, double rpm, double spin) {
        distanceToReferenceMap.put(distance, new ShooterSubsystem.Reference(Rotation2d.fromDegrees(degrees), RPM.of(rpm), spin));
    }

    public ShooterSubsystem.Reference getReferenceByDistance(double distance) {
        return distanceToReferenceMap.get(distance);
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
