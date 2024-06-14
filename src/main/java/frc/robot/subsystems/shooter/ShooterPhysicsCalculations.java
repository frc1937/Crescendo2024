package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.AlliancePose2d;

import static frc.robot.Constants.VisionConstants.BLUE_SPEAKER;
import static frc.robot.Constants.VisionConstants.RED_SPEAKER;
import static frc.robot.subsystems.shooter.ShooterConstants.GRAVITY_FORCE;
import static frc.robot.subsystems.shooter.ShooterConstants.PIVOT_POINT_X_OFFSET_METRES;
import static frc.robot.subsystems.shooter.ShooterConstants.PIVOT_POINT_Z_OFFSET_METRES;
import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_LENGTH_METRES;

public class ShooterPhysicsCalculations {
    private final ShooterSubsystem shooterSubsystem;
    private final double tangentialVelocity;

    private Rotation2d targetPitch = Rotation2d.fromDegrees(20);
    private Pose2d robotPose = new Pose2d();
    private Pose3d targetPose = new Pose3d();
    private double previousDistanceToTarget = 0;

    private final StructArrayPublisher<Pose3d> physicsTargetPose = NetworkTableInstance.getDefault().getStructArrayTopic("lolll", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> poses1 = NetworkTableInstance.getDefault().getStructArrayTopic("poses1", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> poses2 = NetworkTableInstance.getDefault().getStructArrayTopic("poses2", Pose3d.struct).publish();


    public ShooterPhysicsCalculations(ShooterSubsystem shooterSubsystem, double tangentialVelocity) {
        this.shooterSubsystem = shooterSubsystem;
        this.tangentialVelocity = tangentialVelocity;
    }

    public Rotation2d getTargetAnglePhysics() {
        return targetPitch;
    }

    /**
     * We want to update the angle as frequently as possible for less approximation error.
     * Thus, we will run this periodically every time we run a command which uses the physics angle to slowly approach the correct value.
     */
    public void updateValuesForSpeakerAlignment(ChassisSpeeds robotSpeed) {
        boolean isBlueAlliance = AlliancePose2d.AllianceUtils.isBlueAlliance();

        targetPose = isBlueAlliance ? BLUE_SPEAKER : RED_SPEAKER;
//        targetPose = getNewTargetFromRobotVelocity(robotSpeed);
        final Pose3d noteExitPose = getNoteExitPose();
        previousDistanceToTarget = getDistanceToTarget(noteExitPose);

        targetPitch = getPitchPhysics(noteExitPose);
    }

    /**
     * This is seperated from {@link  #updateValuesForSpeakerAlignment(ChassisSpeeds)} for robot pose updates separately;
     *
     * @param robotPose - The robot pose from the correct alliance
     */
    public void feedRobotPose(Pose2d robotPose) {
        this.robotPose = robotPose;
    }

    public Pose3d getTargetPose() {
        return targetPose;
    }

    public Rotation2d getTargetPitch() {
        return targetPitch;
    }

    public double getTangentialVelocity() {
        return tangentialVelocity;
    }

    /**
     * Get the needed pitch theta for the pivot using physics.
     *
     * @return - the required pitch angle
     * @link <a href="https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">Formula is taken from wikipedia</a>
     */
    private Rotation2d getPitchPhysics(Pose3d noteExitPose) {
        double vSquared = tangentialVelocity * tangentialVelocity;

        double z = (targetPose.getZ()) - noteExitPose.getZ();
        double distance = previousDistanceToTarget;

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
     * Returns the required angle the robot has to rotate to face the target
     *
     * @return - the target azimuth angle
     */
    public Rotation2d getAzimuthAngleToTarget() {
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d differenceInXY = targetPose.toPose2d().getTranslation().minus(robotTranslation);

        return Rotation2d.fromRadians(Math.atan2(
                differenceInXY.getY(),
                differenceInXY.getX()));
    }

    /**
     * We assume the robot isn't moving to get the time of flight
     *
     * @return - the time of flight in seconds
     */
    private double getTimeOfFlight() {
        Rotation2d theta = shooterSubsystem.getPitchGoal();

        double distance = previousDistanceToTarget;
        double speed = tangentialVelocity * theta.getCos();

        return distance / speed;
    }

    /**
     * Get the target's offset after taking into account the robot's velocity
     *
     * @param robotVelocity - The robot's current velocity
     * @return - The new pose of the target, after applying the offset
     */
    private Pose3d getNewTargetFromRobotVelocity(ChassisSpeeds robotVelocity) {
        double timeOfFlight = getTimeOfFlight();

        Transform3d targetOffset = new Transform3d(robotVelocity.vxMetersPerSecond * timeOfFlight, robotVelocity.vyMetersPerSecond * timeOfFlight, 0, new Rotation3d());

        return targetPose.transformBy(targetOffset.inverse());
    }

    /**
     * Get the distance from the NOTE's exit position to the target
     *
     * @return - The distance in metres
     */
    private double getDistanceToTarget(Pose3d noteExitPose) {
        return noteExitPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    /**
     * Get the field-relative end of the shooter, AKA the NOTE's point of exit, from field-relative robot pose.
     * Using the correct alliance.
     *
     * @return the shooter's end, AKA the NOTE's point of exit
     */
    private Pose3d getNoteExitPose() {
        Pose3d robotPose3d = new Pose3d(new Pose2d(robotPose.getTranslation(), getAzimuthAngleToTarget()));

        Pose3d robotToPivot = new Pose3d(PIVOT_POINT_X_OFFSET_METRES, 0, PIVOT_POINT_Z_OFFSET_METRES,
                new Rotation3d(0, -shooterSubsystem.getPitchGoal().getRadians(), 0));

        poses1.set(
                new Pose3d[]{robotPose3d.transformBy(robotToPivot.minus(new Pose3d()))}
        );

        Transform3d pivotToShooterEnd = new Transform3d(SHOOTER_LENGTH_METRES, 0, 0, new Rotation3d());

        Pose3d shooterEndPose = robotToPivot.plus(pivotToShooterEnd);
        Transform3d robotToShooterEnd = shooterEndPose.minus(new Pose3d());

        physicsTargetPose.set(
                new Pose3d[] {
                        robotPose3d.transformBy(robotToShooterEnd)
                }
        );

        return robotPose3d.transformBy(robotToShooterEnd);
    }
}
