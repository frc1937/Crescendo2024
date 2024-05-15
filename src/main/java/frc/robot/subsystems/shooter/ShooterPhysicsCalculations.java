package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterPhysicsCalculations {
    private final ShooterSubsystem shooterSubsystem;

    public ShooterPhysicsCalculations(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    /**
     * Get the needed pitch theta for the pivot using physics.
     * @link <a href="https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">Formula is taken from wikipedia</a>
     * @param robotPose - The robot's pose, using the correct alliance
     * @param targetPose - the target pose to hit, using the correct alliance
     * @param tangentialVelocity - the tangential velocity of the flywheel
     * @return - the required pitch angle
     */
    public Rotation2d getPitchAnglePhysics(Pose2d robotPose, Pose3d targetPose, double tangentialVelocity) {
        double g = 9.8;
        double vSquared = tangentialVelocity * tangentialVelocity;

        //This is the distance of the pivot off the floor when parallel to the ground
        double z = targetPose.getZ() - getNoteExitPoseRobodox599(robotPose, targetPose).getZ();
        double distance = getDistanceToTarget(robotPose, targetPose);

        double theta = Math.atan(
                (vSquared + Math.sqrt(vSquared*vSquared - g*(g*distance*distance + 2*vSquared*z))) / (g*distance)
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
        Rotation2d theta = getPitchAnglePhysics(robotPose, targetPose, tangentialVelocity);
        return getDistanceToTarget(robotPose, targetPose) / (tangentialVelocity * theta.getCos());
    }

    /**
     * Returns the required angle the robot has to rotate to face the target
     * @param robotPose - The robot's pose, using the correct alliance
     * @param targetPose - The target pose, using the correct alliance
     * @return - the target azimuth angle
     */
    public Rotation2d getAzimuthAngleToTarget(Pose2d robotPose, Pose3d targetPose) {
        Translation2d differenceInXY = targetPose.toPose2d().getTranslation().minus(robotPose.getTranslation());
        return Rotation2d.fromRadians(Math.atan2(Math.abs(differenceInXY.getY()), Math.abs(differenceInXY.getY())));
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

    /**
     * Get the distance from the note's exit position to the target
     * @param robotPose - The robot's pose, using the correct alliance
     * @param targetPose - The target pose, using the correct alliance
     * @return - The distance in metres
     */
    private double getDistanceToTarget(Pose2d robotPose, Pose3d targetPose) {
        return getNoteExitPoseRobodox599(robotPose, targetPose).getTranslation().getDistance(targetPose.getTranslation());
    }

    /**
     * Get the field-relative end of the shooter, AKA the note's point of exit, from field-relative robot pose.
     * Using the correct alliance.
     * @param robotPose - The robot's pose, using the correct alliance
     * @param targetPose - The target's pose, using the correct alliance
     * @return the shooter's end, AKA the note's existion point
     */
    private Pose3d getNoteExitPoseRobodox599(Pose2d robotPose, Pose3d targetPose) {
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

    /** 71 עד 140
     * תנועת המרי העברי - סיבות הקמת התנועה, פעולות התנועה(ליל הגשרים, הרכבות, פריצה למחנה).
     * העפלה והתיישבות - אקסודוס(העפלה79), התיישבות(11 נק בנגב). קשיים בהעפלה
     * העברת שאלת א"י לאו"ם - סיבות(למה), אינטרסים(ארה"ב ברה"מ)
     * תוכנית החלוקה - הישוב היהודי(בעד ונגד).
     *  מלחמת העצמאות - 6 מאפיינים. 4 ,תקופות
     *  ללמוד רק אירוע אחד בכל תקופה!!!!
     * תקופה 1 - מאפיינים. ערים מעורבות, חזית ועורף. מלחמת מגננה. (שיירת ל"ה!) תכנית ד.
     * תקופה 2 - מעבר מהגנה להתקפה. אירועי כפר יסין. הקרב על ירושלים. העברת אספקה. (מבצע נחשון!)
     * +_  תקופה 3 - הכרזה על הקמת המדינה. (בעד ונגד הטיימינג). הצטרפות של 5 צבאות ערב. (הקרב על קיבו. ניצנים!)+ הפוגה
     *  תקופה 4 - צהל כובש ומשתלט על נקודות ישוב.  כיבוש אילת.(מבצע עובדה!)
     *  הקמת צהך + קשיים + פרשת אלטלנה.
     */
}
