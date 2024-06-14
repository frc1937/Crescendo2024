package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.util.AlliancePose2d;
import org.photonvision.PhotonPoseEstimator;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static frc.lib.util.AlliancePose2d.AllianceUtils.fromCorrectPose;

public final class Constants {
    public static boolean IS_TUNING_MODE = true;

    public static final double POSE_HISTORY_DURATION = 0.3;

    public static final class CanIDConstants {
        public static final int MOUNT_RIGHT_MOTOR_ID = 7;
        public static final int MOUNT_LEFT_MOTOR_ID = 12;

        public static final int INTAKE_MOTOR_ID = 5;

        public static final int PIGEON_ID = 30;

        public static final int
                FLYWHEEL_LEFT_ID = 16,
                FLYWHEEL_RIGHT_ID = 15,
                PIVOT_ID = 1,
                PIVOT_CAN_CODER = 22,
                KICKER_ID = 8;
    }

    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;

    public static final double ROBORIO_LOOP_TIME_SECONDS = 0.02;

    public static final Measure<Distance> FIELD_LENGTH_METRES = Meters.of(16.48);
    public static final Measure<Distance> FIELD_WIDTH_METRES = Meters.of(8.02);

    /**
     * Once how much time, in Hertz (1/hertz = seconds), to run the infrequent periodic procedure
     */
    public static final double INFREQUENT_PERIODIC_HERTZ = 10,
            PERIODIC_FREQUENCY_HERTZ = 50,
            ODOMETRY_FREQUENCY_HERTZ = 200;

    public static final double STICK_DEADBAND = 0.1;

    public static final double DRIVE_NEUTRAL_DEADBAND = 0.2;
    public static final double ROTATION_NEUTRAL_DEADBAND = 0.2;

    public static final class Mount { //todo: TUNE CONSTANTS TO SYSTEM SPECIFICS
        public static final Measure<Angle> MOUNT_AT_TOP_LEFT_VALUE = Rotations.of(68);
        public static final Measure<Angle> MOUNT_AT_TOP_RIGHT_VALUE = Rotations.of(69);

        public static final double MOUNT_SPEED_SCALAR = 0.9;
        public static final double MOUNT_AUTO_SPEED = 0.5;
    }

    public static final class Transforms {
        /**
         * Physical location of the camera on the robot, relative to the center of the robot. NEEDS TUNING
         */
        public static final Transform3d ROBOT_TO_FRONT_CAMERA =
                new Transform3d(
                        0.5, 0.175245/*-0.175245*/, 0.46,
                        new Rotation3d(0, Units.degreesToRadians(-36+10), 0)
                );
    }

    public static final class LEDsConstants {
        public static final int LEDS_COUNT = 46;
        public static final Color8Bit COLOUR_WHEN_EMPTY = new Color8Bit(0, 0, 255);
        public static final Color8Bit COLOUR_WHEN_LOADED = new Color8Bit(255, 80, 0);
    }

    public static final class NavigationConstants {
        public static final Pose2d AMPLIFIER_POSE = new Pose2d(1.83, 7.7, Rotation2d.fromDegrees(-90));
        public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(2.7, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));
    }


    public static class VisionConstants {
        public static final String FRONT_CAMERA_NAME = "Front1937";
        public static final String REAR_CAMERA_NAME = "Rear1937";
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        /**
         * Minimum target ambiguity. Targets with higher ambiguity will be discarded
         */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

        /**
         * The vector represents how ambiguous each value is.
         * The first value represents how ambiguous is the x,
         * the second one for the y, and the third one is for the theta (rotation).
         * Increase these numbers to trust the odometry less.
         */
        public static final Vector<N3> STATES_AMBIGUITY = VecBuilder.fill(0.003, 0.003, 0.0002);
        public static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

        public static final AlliancePose2d DEFAULT_POSE = fromCorrectPose(0, 0, Rotation2d.fromDegrees(180));

        public static final double TRANSLATION_STD_EXPONENT = 0.005;
        public static final double ROTATION_STD_EXPONENT = 0.01;

        public static final Map<Integer, Pose3d> TAG_ID_TO_POSE = new HashMap<>();

        static {
            for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
                TAG_ID_TO_POSE.put(aprilTag.ID, aprilTag.pose);
        }

        public static final Pose3d BLUE_SPEAKER = VisionConstants.TAG_ID_TO_POSE.get(7).plus(new Transform3d(new Translation3d(0.2, 0.0, 1.3



        ), new Rotation3d()));
        public static final Pose3d RED_SPEAKER = AlliancePose2d.AllianceUtils.mirrorPose(BLUE_SPEAKER);

        public static final PhotonPoseEstimator.PoseStrategy PRIMARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
        public static final PhotonPoseEstimator.PoseStrategy SECONDARY_POSE_STRATEGY = PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
    }
}
