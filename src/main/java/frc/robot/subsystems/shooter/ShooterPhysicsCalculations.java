package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.poseestimation.PoseEstimator5990;

import static frc.robot.subsystems.shooter.ShooterConstants.GRAVITY_FORCE;
import static frc.robot.subsystems.shooter.ShooterConstants.PIVOT_POINT_X_OFFSET_METRES;
import static frc.robot.subsystems.shooter.ShooterConstants.PIVOT_POINT_Z_OFFSET_METRES;
import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_LENGTH_METRES;

public class ShooterPhysicsCalculations {
    private final ShooterSubsystem shooterSubsystem;
    private final PoseEstimator5990 poseEstimator5990;

    private Pose2d robotPose;

    public ShooterPhysicsCalculations(ShooterSubsystem shooterSubsystem, PoseEstimator5990 poseEstimator5990) {
        this.shooterSubsystem = shooterSubsystem;
        this.poseEstimator5990 = poseEstimator5990;

        robotPose = poseEstimator5990.getCurrentPose().getBluePose();
    }

    public void feedRobotPose(Pose2d robotPose) {
        this.robotPose = robotPose;
    }

    /**
     * Get the needed pitch theta for the pivot using physics.
     *
     * @param targetPose         - the target pose to hit, using the correct alliance
     * @param tangentialVelocity - the tangential velocity of the flywheel
     * @return - the required pitch angle
     * @link <a href="https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">Formula is taken from wikipedia</a>
     * //     * @param robotPose - The robot's pose, using the correct alliance
     */
    public Rotation2d getPitchAnglePhysics(Pose3d targetPose, double tangentialVelocity) {
        double vSquared = tangentialVelocity * tangentialVelocity;

        //This is the distance of the pivot off the floor when parallel to the ground
        double z = (targetPose.getZ()) - getNoteExitPose(robotPose, targetPose).getZ();
        double distance = getDistanceToTarget(robotPose, targetPose);

        double sqrt = Math.sqrt(vSquared * vSquared - GRAVITY_FORCE * (GRAVITY_FORCE * distance * distance + 2 * vSquared * z));
        double theta = Math.atan((vSquared - sqrt) / (GRAVITY_FORCE * distance));

        SmartDashboard.putNumber("physics/DivNumerator", (vSquared + sqrt));
        SmartDashboard.putNumber("physics/DivDenominator", GRAVITY_FORCE * distance);
        SmartDashboard.putNumber("physics/DivResult", (vSquared + sqrt) / (GRAVITY_FORCE * distance));
        SmartDashboard.putNumber("physics/ZedDistance", z);
        SmartDashboard.putNumber("physics/distance", distance);
        SmartDashboard.putString("physics/robotPose", robotPose.toString());
        SmartDashboard.putNumber("physics/FunctionTheta", theta);

        return Rotation2d.fromRadians(theta);
    }

    /**
     * We assume the robot isn't moving to get the time of flight
     * //     * @param robotPose - The robot's pose, using the correct alliance
     *
     * @param targetPose         - The target pose of the NOTE, using the correct alliance
     * @param tangentialVelocity - The tangential velocity of the flywheels
     * @return - the time of flight in seconds
     */
    public double getTimeOfFlight(Pose2d robotPose, Pose3d targetPose, double tangentialVelocity) {
        Rotation2d theta = getPitchAnglePhysics(targetPose, tangentialVelocity);

        double distance = getDistanceToTarget(robotPose, targetPose);
        double speed = tangentialVelocity * theta.getCos();

        return distance / speed;
    }

    /**
     * Returns the required angle the robot has to rotate to face the target
     *
     * @param targetPose - The target pose, using the correct alliance
     * @return - the target azimuth angle
     */
    public Rotation2d getAzimuthAngleToTarget(Pose3d targetPose) {
        Translation2d robotTranslation = poseEstimator5990.getCurrentPose().getBluePose().getTranslation();
        Translation2d differenceInXY = targetPose.toPose2d().getTranslation().minus(robotTranslation);

        return Rotation2d.fromRadians(Math.atan2(Math.abs(differenceInXY.getY()), Math.abs(differenceInXY.getY())));
    }

    /**
     * Get the target's offset after taking into account the robot's velocity
     *
     * @param robotPose          - The robot's pose, using the correct alliance
     * @param targetPose         - The target pose, using the correct alliance
     * @param tangentialVelocity - The tangential velocity of the flywheels
     * @param robotVelocity      - The robot's current velocity
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

    /**
     * Get the distance from the NOTE's exit position to the target
     *
     * @param robotPose  - The robot's pose, using the correct alliance
     * @param targetPose - The target pose, using the correct alliance
     * @return - The distance in metres
     */
    private double getDistanceToTarget(Pose2d robotPose, Pose3d targetPose) {
        return getNoteExitPose(robotPose, targetPose).getTranslation()
                .getDistance(targetPose.getTranslation());
    }

    /**
     * Get the field-relative end of the shooter, AKA the NOTE's point of exit, from field-relative robot pose.
     * Using the correct alliance.
     *
     * @param robotPose  - The robot's pose, using the correct alliance
     * @param targetPose - The target's pose, using the correct alliance
     * @return the shooter's end, AKA the NOTE's point of exit
     */
    private Pose3d getNoteExitPose(Pose2d robotPose, Pose3d targetPose) {
        Pose3d robotPose3d = new Pose3d(new Pose2d(robotPose.getTranslation(), targetPose.getRotation().toRotation2d()));

        Transform3d robotToPivot = new Transform3d(
                PIVOT_POINT_X_OFFSET_METRES, 0, PIVOT_POINT_Z_OFFSET_METRES,
                new Rotation3d(0, -shooterSubsystem.getPitchGoal().getRadians(), 0)
        );

        Transform3d pivotToShooterEnd = new Transform3d(SHOOTER_LENGTH_METRES, 0, 0, new Rotation3d());

        Pose3d shooterEndPose = new Pose3d().transformBy(robotToPivot).plus(pivotToShooterEnd);
        Transform3d robotToShooterEnd = shooterEndPose.minus(new Pose3d());

        return robotPose3d.transformBy(robotToShooterEnd);
    }
}
