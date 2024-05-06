package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShooterPhysicsCalculations {
    private final ShooterSubsystem shooterSubsystem;

    public ShooterPhysicsCalculations(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    /**
     * Get the needed pitch theta for the pivot using physics.
     * <p>Formula is taken from
     * <a href="https://en.wikipedia.org/wiki/Projectile_motion?useskin=vector#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">Projectile Motion Wikipedia</a>
     * </p>
     * @param robotPose - The robot's pose, using the correct alliance
     * @param targetPose - the target pose to hit, using the correct alliance
     * @param tangentialVelocity - the tangential velocity of the flywheel
     * @return - the required pitch angle
     */
    public Rotation2d getPitchAnglePhysics(Pose2d robotPose, Pose3d targetPose, double tangentialVelocity) {
        Pose3d exitPose = getNoteExitPosition(robotPose);

        double g = 9.8;
        double vSquared = tangentialVelocity * tangentialVelocity;

        //This is the distance of the pivot off the floor when parallel to the ground
        double z = targetPose.getZ() - exitPose.getZ();//Inch.of(8.5).in(Meters);
        double distance = getDistanceToTarget(robotPose, targetPose);

        double theta = Math.atan(
                (vSquared + Math.sqrt(vSquared*vSquared - g*(g*distance*distance + 2*vSquared*z)))
                        / (g*distance)
        );

        return Rotation2d.fromRadians(theta);
    }

    /**
     * We assume the robot isn't moving to get the time of flight
     * @param robotPose - The robot's pose, using the correct alliance
     * @param targetPose - The target pose of the note, using the correct alliance
     * @param tangentialVelocity - The tangential velocity of the flywheels
     * @return - the time of flight in seconds
     */
    public double getTimeOfFlight(Pose2d robotPose, Pose3d targetPose, double tangentialVelocity) {
        Pose3d exitPose = getNoteExitPosition(robotPose);

        Rotation2d theta = getPitchAnglePhysics(robotPose, targetPose, tangentialVelocity);

        double xDiff = targetPose.getX() - exitPose.getX();
        double yDiff = targetPose.getY() - exitPose.getY();

        double distance = Math.hypot(xDiff, yDiff);

        return distance / (tangentialVelocity * theta.getCos());
    }

    /**
     * Returns the required angle the robot has to rotate to face the target
     * @param robotPose - The robot's pose, using the correct alliance
     * @param targetPose - The target pose, using the correct alliance
     * @return - the target azimuth angle
     */
    public Rotation2d getAzimuthAngleToTarget(Pose2d robotPose, Pose3d targetPose) {
        Translation2d difference = targetPose.toPose2d().getTranslation().minus(robotPose.getTranslation());
        return Rotation2d.fromRadians(Math.atan2(Math.abs(difference.getY()), Math.abs(difference.getY())));
    }

    /**
     * Get the target's offset after taking into account the robot's velocity
     * @param robotPose - The robot's pose, using the correct alliance
     * @param targetPose - The target pose, using the correct alliance
     * @param tangentialVelocity - The tangential velocity of the flywheels
     * @param robotVelocity - The robot's current velocity
     * @return - The new pose of the target, after applying the offset
     */
    public Pose3d getNewTargetFromRobotVelocity(Pose2d robotPose, Pose3d targetPose, double tangentialVelocity, ChassisSpeeds robotVelocity) {
        double timeOfFlight = getTimeOfFlight(robotPose, targetPose, tangentialVelocity);

        Transform3d targetOffset = new Transform3d(
                robotVelocity.vxMetersPerSecond * timeOfFlight,
                robotVelocity.vyMetersPerSecond * timeOfFlight,
                0,
                new Rotation3d()
        );

        return targetPose.transformBy(targetOffset.inverse());
    }

    private double getDistanceToTarget(Pose2d robotPose, Pose3d targetPose) {
//        return robotPose.minus(targetPose.toPose2d()).getTranslation().getNorm();
//        return Math.hypot(targetPose.getX() - robotPose.getX(), targetPose.getY() - robotPose.getY());
        return robotPose.getTranslation().getDistance(targetPose.toPose2d().getTranslation());
        //todo: TEST WHETHER THESE ARE ALL THE ~SAME~
    }

    /**
     * Get the field-relative end of the shooter, AKA the note's point of exit, from field-relative robot pose.
     * Using the correct alliance.
     * @param robotPose - The robot's pose, using the correct alliance
     */
    private Pose3d getNoteExitPosition(Pose2d robotPose) {
        Rotation2d pitchAngle = shooterSubsystem.getPitchGoal();

        Transform3d pivotFromRobot = new Transform3d(
                0,
                0,
                0,
                new Rotation3d(0, 0, 0)
        ); //TODO: Tune from CAD

        double pitchLength = 0.5; //TODO: Tune from CAD

        Transform3d pitchEndFromPivot = new Transform3d(
                pitchLength * robotPose.getRotation().getCos(),
                pitchLength * robotPose.getRotation().getCos(),
                pitchLength * Math.sin(pitchAngle.getRadians()),
                new Rotation3d(0, 0, 0)
        ); //todo: This might be completely wrong! XXX TEST

        Pose3d robotPose3d = new Pose3d(robotPose);

        return robotPose3d.transformBy(pivotFromRobot).transformBy(pitchEndFromPivot);
    }
}
