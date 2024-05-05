package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

public class ShooterPhysicsCalculations {
    /**
     * Get the needed pitch theta for the pivot using physics.
     * <p>Formula is taken from
     * <a href="https://en.wikipedia.org/wiki/Projectile_motion?useskin=vector#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">...</a>
     * </p>
     * @param exitPose - The location of exit's pose, using the correct alliance
     * @param targetPose - the target pose to hit, using the correct alliance
     * @param tangentialVelocity - the tangential velocity of the flywheel
     * @return - the required pitch angle
     */
    public static Rotation2d getPitchAnglePhysics(Pose2d exitPose, Pose3d targetPose, double tangentialVelocity) {
        double g = 9.8;
        double vSquared = tangentialVelocity * tangentialVelocity;

        //This is the distance of the pivot off the floor when parallel to the ground
        double z = targetPose.getZ() - Inch.of(8.5).in(Meters);
        double distance = Math.hypot(exitPose.getX() - targetPose.getX(), exitPose.getY() - targetPose.getY());

        double theta = Math.atan(
                (vSquared + Math.sqrt(vSquared*vSquared - g*(g*distance*distance + 2*vSquared*z)))
                        / (g*distance)
        );

        return Rotation2d.fromRadians(theta);
    }

    /**
     * We assume the robot isn't moving to get the time of flight
     * @param exitPose - The location of exit's pose, using the correct alliance
     * @param targetPose - The target pose of the note, using the correct alliance
     * @param tangentialVelocity - The tangential velocity of the flywheels
     * @return - the time of flight in seconds
     */
    public static double getTimeOfFlight(Pose2d exitPose, Pose3d targetPose, double tangentialVelocity) {
        Rotation2d theta = getPitchAnglePhysics(exitPose, targetPose, tangentialVelocity);

        double xDiff = targetPose.getX() - exitPose.getX();
        double yDiff = targetPose.getY() - exitPose.getY();

        double distance = Math.hypot(xDiff, yDiff);

        return distance / (tangentialVelocity * theta.getCos());
    }


    /**
     * Get the field-relative end of the shooter, AKA the note's point of exit, from field-relative robot pose.
     * Using the correct alliance.
     * @param robotPose - The robot's pose, using the correct alliance
     * @param targetPose - The target pose, using the correct alliance
     * @param pitchAngle - The pitch angle of the flywheel
     */
    private void getNoteExitPosition(Pose2d robotPose, Pose3d targetPose, Rotation2d pitchAngle) {
        //todo: make method work
    }
}
