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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.lib.math.Conversions.tangentialVelocityFromRPM;
import static frc.robot.Constants.FIELD_LENGTH_METRES;

public class Target {
    private final Translation2d bluePosition;
    private final Translation2d redPosition;
    private final InterpolatingTreeMap<Double, ShooterSubsystem.Reference> distanceToReferenceMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShooterSubsystem.Reference::interpolate);
    private final InterpolatingDoubleTreeMap distanceToTimeOfFlightMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap distanceToAzimuthToleranceMap = new InterpolatingDoubleTreeMap();

    public Target(Translation2d bluePosition) {
        this.bluePosition = bluePosition;
        redPosition = new Translation2d(FIELD_LENGTH_METRES.in(Meters) - bluePosition.getX(), bluePosition.getY());
    }

    /** Calculate the displacement from the centre of the robot to the target */
    public Translation2d calculateTargetDisplacement(RobotState robotState) {
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Translation2d targetPosition = alliance == DriverStation.Alliance.Red ? redPosition : bluePosition;
        return targetPosition.minus(robotState.getPose().getTranslation());
    }

    public void putMeasurement(double distanceMetres, double degrees, double rpm) {
        distanceToReferenceMap.put(distanceMetres, new ShooterSubsystem.Reference(Rotation2d.fromDegrees(degrees), tangentialVelocityFromRPM(rpm, Units.inchesToMeters(4.0))));
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
