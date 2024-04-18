// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.Constants.ShootingConstants.FIELD_LENGTH;

public class Target {
    private final Translation2d bluePosition;
    private final Translation2d redPosition;
    private final InterpolatingTreeMap<Double, ShooterSubsystem.Reference> distanceToReferenceMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShooterSubsystem.Reference::interpolate);
    private final InterpolatingDoubleTreeMap distanceToTimeOfFlightMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap distanceToAzimuthToleranceMap = new InterpolatingDoubleTreeMap();

    public Target(Translation2d bluePosition) {
        this.bluePosition = bluePosition;
        redPosition = new Translation2d(FIELD_LENGTH - bluePosition.getX(), bluePosition.getY());
    }

    /** Calculate the displacement from the centre of the robot to the target */
    public Translation2d calculateTargetDisplacement(RobotState robotState) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d targetPosition = alliance == DriverStation.Alliance.Red ? redPosition : bluePosition;
        return targetPosition.minus(robotState.getPose().getTranslation());
    }

    public void putMeasurement(double distanceMetres, double degrees, double rpm, double spin) {
        distanceToReferenceMap.put(distanceMetres, new ShooterSubsystem.Reference(Rotation2d.fromDegrees(degrees), RPM.of(rpm), spin));
    }

    public void putTimeOfFlightMeasurement(double distanceMetres, double timeOfFlightSeconds) {
        distanceToTimeOfFlightMap.put(distanceMetres, timeOfFlightSeconds);
    }

    public void putAzimuthToleranceMeasurement(double distanceMetres, double toleranceRadians) {
        distanceToAzimuthToleranceMap.put(distanceMetres, toleranceRadians);
    }

    public ShooterSubsystem.Reference getReferenceByDistance(double distance) {
        return distanceToReferenceMap.get(distance);
    }

    public Measure<Angle> getAzimuthTolerance(double distanceMetres) {
        return Radians.of(distanceToAzimuthToleranceMap.get(distanceMetres));
    }

    /**
     * Find the displacement to the virtual target, i.e., the target to which the robot should aim s.t.
     * the NOTE enters the actual target, even whilst moving.
     */
    public Translation2d calculateVirtualTargetDisplacement(double virtualTargetDistance,
                                                            Translation2d targetDisplacement,
                                                            ChassisSpeeds velocity) {
        double timeOfFlight = distanceToTimeOfFlightMap.get(virtualTargetDistance);

        Translation2d displacementDueToRobotVelocity = new Translation2d(
                velocity.vxMetersPerSecond * timeOfFlight,
                velocity.vyMetersPerSecond * timeOfFlight
        );

        return targetDisplacement.minus(displacementDueToRobotVelocity);
    }
}
