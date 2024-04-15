package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class SwerveOptimization {
    /**
     * When changing direction, the module will skew since the angle motor is not at its target angle.
     * This method will counter that by reducing the target velocity according to the angle motor's error cosine.
     *
     * @param targetVelocity the target velocity, in meters per second
     * @param targetSteerAngle              the target steer angle
     * @return the reduced target velocity in revolutions per second
     */
    public Measure<Velocity<Distance>> reduceSkew(Measure<Velocity<Distance>> targetVelocity, Rotation2d targetSteerAngle, Rotation2d currentAngle) {
        final double closedLoopError = targetSteerAngle.getRadians() - currentAngle.getRadians();
        final double cosineScalar = Math.abs(Math.cos(closedLoopError));
        return MetersPerSecond.of(targetVelocity.in(MetersPerSecond) * cosineScalar);
    }
}
