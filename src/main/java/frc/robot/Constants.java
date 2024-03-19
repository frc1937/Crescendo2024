package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.Target;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static final String TFILAT_HADERECH = """
            May it be Your will, Lord, our God and the God of our ancestors, that You lead us toward
            peace, guide our footsteps toward peace, that we are supported in peace, and make us reach
            our desired destination for life, gladness, and peace. May You rescue us from the hand of
            every foe and ambush, from robbers and wild beasts on the trip, and from all manner of
            punishments that assemble to come to earth. May You send blessing in our handiwork, and
            grant us grace, kindness, and mercy in Your eyes and in the eyes of all who see us. May
            You hear the sound of our humble request because You are God Who hears prayer requests.
            Blessed are You, Lord, Who hears prayer""";

    /**
     * Once how much time, in seconds, to run the infrequent periodic procedure
     */
    public static final double INFREQUENT_PERIODIC_PERIOD = 0.05;
    public static final double STICK_DEADBAND = 0.1;

    public static final class Mount { //todo: TUNE CONSTANTS TO SYSTEM SPECIFICS
        public static final int MOUNT_RIGHT_MOTOR_ID = 2;
        public static final int MOUNT_LEFT_MOTOR_ID = 3;
        public static final Measure<Angle> MOUNT_AT_TOP_ENCODER_VALUE = Degrees.of(0.5);
        public static final Measure<Angle> MOUNT_SOFT_LIMIT = Degrees.of(1.5);
        public static final double MOUNT_SPEED_SCALAR = 0.5;
        public static final double MOUNT_AUTO_SPEED = 0.5;
    }


    public static final class Transforms {
        /**
         * Physical location of the camera on the robot, relative to the center of the robot. NEEDS TUNING
         */
        public static final Transform3d
                FRONT_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(-0.31, -0.13, 0.27), new Rotation3d(0, Units.degreesToRadians(-25), 0)),
                ROBOT_TO_FRONT_CAMERA = FRONT_CAMERA_TO_ROBOT.inverse();
        public static final Transform3d
                REAR_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.355, 0.11, 0.41),
                new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(180))),
                ROBOT_TO_REAR_CAMERA = REAR_CAMERA_TO_ROBOT.inverse();
        public static final Translation3d ROBOT_TO_PIVOT = new Translation3d(-0.275, 0, 0.285);
    }

    public static class VisionConstants {
        public static final String FRONT_CAMERA_NAME = "Front1937";
        public static final String REAR_CAMERA_NAME = "Rear1937";
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        /**
         * Minimum target ambiguity. Targets with higher ambiguity will be discarded
         */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
        public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
        public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
        public static final double NOISY_DISTANCE_METERS = 2.5;
        public static final double DISTANCE_WEIGHT = 7;
        public static final int TAG_PRESENCE_WEIGHT = 10;


        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision less.
         * This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
         */
        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(), 1.d, 1.d, 1.d);

        /**
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This
         * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
         * meters.
         */
        public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(), 0.1, 0.1, 0.03);
    }

    public static class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 11;
    }

    public static final class ShootingConstants {
        /**
         * An initial presumption to the slope of the path from the robot's
         * shooter to the virtual target
         * <p>
         * Since the slope to the virtual target depends on the time of flight,
         * which itself depends on the slope to the virtual target, we provide
         * an initial guess.
         * <p>
         * The value hereby is arbitrary and it depicts a common value for the
         * slope.
         */
        public static final double DEFAULT_SLOPE_TO_VIRTUAL_TARGET = 0.5;
        public static final int FLYWHEEL_MAX_RPM = 6400;
        public static final InterpolatingTreeMap<Double, Double> DISTANCE_TO_TIME_OF_FLIGHT_MAP = new InterpolatingTreeMap<>(
                InverseInterpolator.forDouble(), Interpolator.forDouble());

        static {
            DISTANCE_TO_TIME_OF_FLIGHT_MAP.put(0.87, 0.35);
            DISTANCE_TO_TIME_OF_FLIGHT_MAP.put(0.67, 0.4);
            DISTANCE_TO_TIME_OF_FLIGHT_MAP.put(0.52, 0.5);
            DISTANCE_TO_TIME_OF_FLIGHT_MAP.put(0.41, 0.6);
        }

        public static final double POSE_HISTORY_DURATION = 0.3;

        public static final double PITCH_INTAKE_FLOOR_ANGLE = -21.7;
        public static final double PITCH_INTAKE_FEEDER_ANGLE = 51;
        public static final Rotation2d PITCH_DEFAULT_ANGLE = Rotation2d.fromDegrees(PITCH_INTAKE_FLOOR_ANGLE);

        public static final int
                FLYWHEEL_LEFT_ID = 16,
                FLYWHEEL_RIGHT_ID = 15,
                PIVOT_ID = 1,
                PIVOT_CAN_CODER = 22,
                KICKER_ID = 8;
        public static final double PIVOT_ENCODER_OFFSET = 343;
        public static final double PITCH_KS = 0.38398,
                PITCH_KG = 0.31481,
                PITCH_KV = 12.824,
                PITCH_KA = 3.995,
                PITCH_KP = 1.5,//1.d / 0.02,
                PITCH_KD = 0.0,
                PITCH_MAX_VELOCITY = 25,//1.05,
                PITCH_MAX_ACCELERATION = 35;//0.75;
        public static final float PIVOT_CONSTRAINT_DEGREES = 130;  // TODO This is not the final value
        public static final double PIVOT_TOLERANCE = Units.degreesToRadians(1);
        public static final CANSparkBase.SoftLimitDirection PIVOT_CONSTRAINT_DIRECTION = CANSparkBase.SoftLimitDirection.kForward;
        public static final float PITCH_TRANSMISSION_RATIO = 150;
        public static final double DEFAULT_PITCH_DEADBAND = 0.025,
                VERTICAL_PITCH_DEADBAND = 0.06;

        public static final class FlywheelControlConstants {
            /**
             * All in rotations per second and voltages
             */
            public static final double
                    RIGHT_P = 0.027796,
                    RIGHT_S = 0.025648,
                    RIGHT_V = 0.10953,
                    RIGHT_A = 0.02502,
                    LEFT_P = 0.03,  // NOTE: This value was not provided by SysId
                    LEFT_S = 0.2426,
                    LEFT_V = 0.11243,
                    LEFT_A = 0.014245,
                    TOLERANCE = 3;
        }

        /**
         * In seconds
         */
        public static final double SHOOTING_DELAY = 0.5;
        /**
         * In seconds
         */
        public static final double POST_SHOOTING_DELAY = 0.25;
        public static final double SHOOTING_PREDICTION_TIME = 0.5;

        public static final Target BLUE_SPEAKER_TARGET = new Target(new Translation2d(0.03, 5.3), Degrees.of(1.85));
        public static final Target BLUE_ASSIST_TARGET = new Target(new Translation2d(1.5, 7.0), Degrees.of(3.5));
        public static final double FIELD_LENGTH = 16.48;

        static {
            BLUE_SPEAKER_TARGET.putMeasurement(1.24, 53, 2500, 1);
            BLUE_SPEAKER_TARGET.putMeasurement(1.56, 47, 2500, 1);
            BLUE_SPEAKER_TARGET.putMeasurement(1.85, 44, 2500, 1);
            BLUE_SPEAKER_TARGET.putMeasurement(2.08, 40, 2700, 1);
            BLUE_SPEAKER_TARGET.putMeasurement(2.33, 35, 3000, 0.85);
            BLUE_SPEAKER_TARGET.putMeasurement(2.72, 34, 3000, 0.85);
            BLUE_SPEAKER_TARGET.putMeasurement(2.95, 31, 3500, 0.85);
            BLUE_SPEAKER_TARGET.putMeasurement(3.27, 27.8, 3500, 0.85);
            BLUE_SPEAKER_TARGET.putMeasurement(3.7, 26.1, 4000, 0.8);
            BLUE_SPEAKER_TARGET.putMeasurement(4.1, 22.9, 5000, 0.9);
            BLUE_SPEAKER_TARGET.putMeasurement(4.35, 21.8, 5400, 0.9);
            BLUE_SPEAKER_TARGET.putMeasurement(4.7, 21.5, 5500, 0.9);
            BLUE_SPEAKER_TARGET.putMeasurement(4.82, 20.235, 6150, 1.1);
        }

        static {
            BLUE_ASSIST_TARGET.putMeasurement(5, 45, 1700, 0.7);
            BLUE_ASSIST_TARGET.putMeasurement(8, 45, 2300, 0.75);
            BLUE_ASSIST_TARGET.putMeasurement(11, 36, 2900, 0.8);
            BLUE_ASSIST_TARGET.putMeasurement(14, 36, 3500, 0.83);
        }

        public static final double KICKER_SPEED_BACKWARDS = -0.7;
        public static final double KICKER_SPEED_FORWARD = 1;
        public static final double CONSIDERED_NOISELESS_THRESHOLD = 20;
        public static final ShooterSubsystem.Reference SPEAKER_FRONT = new ShooterSubsystem.Reference(
                Rotation2d.fromDegrees(50), RPM.of(2500));//50!
        public static final ShooterSubsystem.Reference SPEAKER_BACK = new ShooterSubsystem.Reference(
                Rotation2d.fromDegrees(113), RPM.of(2000));

        public static final ShooterSubsystem.Reference INTAKE = new ShooterSubsystem.Reference(
                Rotation2d.fromDegrees(-21.2), RPM.of(-1800), 0.75, 16);

        public static final ShooterSubsystem.Reference AMP_INIT =
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(102), RPM.of(500));
    }

    public static final class Swerve {
        public static final double AZIMUTH_CONTROLLER_P = 9.4, AZIMUTH_CONTROLLER_I = 0,
                AZIMUTH_CONTROLLER_D = 0, AZIMUTH_CONTROLLER_TOLERANCE = Units.degreesToRadians(1.85),
                AZIMUTH_CONTROLLER_DEADBAND = 0.1;
        public static final double TRANSLATION_CONTROLLER_P = 1.366, TRANSLATION_MAX_VELOCITY = 2.7, TRANSLATION_MAX_ACCELERATION = 3;
        public static final TrapezoidProfile.Constraints TRANSLATION_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION);

        public static final int PIGEON_ID = 30;
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants CHOSEN_MODULE =
                COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.615;
        public static final double WHEEL_BASE = 0.565;
        public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;
        public static final Measure<Distance> DRIVE_BASE_RADIUS =
                Meters.of(new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2).getNorm());

        /* SwerveSubsystem Kinematics
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = false;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERT = CHOSEN_MODULE.canCoderInvert;

        /* SwerveSubsystem Current Limiting */
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KFF = 0;
        public static final double ANGLE_KP = 0.01;
        public static final double ANGLE_KI = 0;
        public static final double ANGLE_KD = 0;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.053067;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values */
        // Convert the values calculated by SysId from volts to [-1, 1], which TalonFX uses.
        public static final double DRIVE_KS = 0.27053 / 12.d;
        public static final double DRIVE_KV = 0.10861 / 12.d;
        public static final double DRIVE_KA = 0.023132 / 12.d;

        /* SwerveSubsystem Profiling Values */
        /**
         * Radians per Second
         */
        public static final double MAX_ANGULAR_VELOCITY = 9.2;  // up to 9.5 in the start of the match
        /**
         * Meters per Second
         */
        public static final double MAX_SPEED = 5;  //MAX_ANGULAR_VELOCITY / DRIVE_BASE_RADIUS.in(Meters);

        /* Neutral Modes */
        public static final com.revrobotics.CANSparkBase.IdleMode ANGLE_NEUTRAL_MODE = CANSparkBase.IdleMode.kBrake;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Module0 {
            public static final int DRIVE_MOTOR_ID = 14;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final int CAN_CODER_ID = 18;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(246.885);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Module1 {
            public static final int DRIVE_MOTOR_ID = 3;
            public static final int ANGLE_MOTOR_ID = 10;
            public static final int CAN_CODER_ID = 20;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(100.898);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Module2 {
            public static final int DRIVE_MOTOR_ID = 13;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final int CAN_CODER_ID = 19;//
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(190.459);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Module3 {
            public static final int DRIVE_MOTOR_ID = 2;
            public static final int ANGLE_MOTOR_ID = 9;
            public static final int CAN_CODER_ID = 21;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(115.840);
            public static final SwerveModuleConstants CONSTANTS =
                    new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        public static final boolean ANGLE_INVERT = true;
        public static final double VOLTAGE_COMP = 12.0;
        public static final double ANGLE_CONVERSION_FACTOR = 360.0 / ANGLE_GEAR_RATIO;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;
        public static final TrapezoidProfile.Constraints AZIMUTH_CONTROLLER_CONSTRAINTS =
                new TrapezoidProfile.Constraints(
                        MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        /**
         * We avoid steering the swerve wheels below a certain driving speed, for in-place turning
         * causes them to jitter. Thus, we hereby define the maximum driving speed that is
         * considered 'in-place'.
         */
        public static final double SWERVE_IN_PLACE_DRIVE_MPS = 0.01 * MAX_SPEED;

        public static final class AutoConstants {
            public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
                    new PIDConstants(TRANSLATION_CONTROLLER_P, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(AZIMUTH_CONTROLLER_P, AZIMUTH_CONTROLLER_I, AZIMUTH_CONTROLLER_D), // Rotation PID constants
                    Constants.Swerve.MAX_SPEED,
                    DRIVE_BASE_RADIUS.in(Meters),
                    new ReplanningConfig(true, true));
        }
    }

    public static final class NavigationConstants {
        public static final Pose2d AMPLIFIER_POSE = new Pose2d(1.83, 7.7, Rotation2d.fromDegrees(-90));
        public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(2.7, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));
    }
}
