package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import static edu.wpi.first.units.Units.Meters;

public final class SwerveConstants {
    public static final double AZIMUTH_CONTROLLER_P = 9.4, AZIMUTH_CONTROLLER_I = 0,
            AZIMUTH_CONTROLLER_D = 0, AZIMUTH_CONTROLLER_TOLERANCE = Units.degreesToRadians(1.85),
            AZIMUTH_CONTROLLER_DEADBAND = 0.12;
    public static final double TRANSLATION_CONTROLLER_P = 1.366, TRANSLATION_MAX_VELOCITY = 2.7, TRANSLATION_MAX_ACCELERATION = 3;
    public static final TrapezoidProfile.Constraints TRANSLATION_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION);

    public static final int PIGEON_ID = 30;
    public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants CHOSEN_MODULE =
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = 0.615;
    public static final double WHEEL_BASE = 0.565;
    public static final Measure<Distance> WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;
    public static final Measure<Distance> DRIVE_BASE_RADIUS =
            Meters.of(new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2).getNorm());

    public static final Measure<Distance> WHEEL_DIAMETER_METRES = CHOSEN_MODULE.wheelDiameter;
    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;

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
    public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;//false;
    //TODO XXX WARNING: Check if CounterCLockwise is IN FACT the equivalent of false!!!

    /* Angle Encoder Invert */
    public static final SensorDirectionValue CAN_CODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;// CHOSEN_MODULE.canCoderInvert;
    //TODO XXX WARNING: Check if CounterCLockwise is IN FACT the equivalent of false!!!

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
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Module0 {
        public static final int DRIVE_MOTOR_ID = 14;
        public static final int ANGLE_MOTOR_ID = 11;
        public static final int CAN_CODER_ID = 18;
        public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(246.885);
        public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(0, DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
    }

    /* Front Right Module - Module 1 */
    public static final class Module1 {
        public static final int DRIVE_MOTOR_ID = 3;
        public static final int ANGLE_MOTOR_ID = 10;
        public static final int CAN_CODER_ID = 20;
        public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(100.898);
        public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(1, DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
    }

    /* Back Left Module - Module 2 */
    public static final class Module2 {
        public static final int DRIVE_MOTOR_ID = 13;
        public static final int ANGLE_MOTOR_ID = 6;
        public static final int CAN_CODER_ID = 19;//
        public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(190.459);
        public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(2, DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
    }

    /* Back Right Module - Module 3 */
    public static final class Module3 {
        public static final int DRIVE_MOTOR_ID = 2;
        public static final int ANGLE_MOTOR_ID = 9;
        public static final int CAN_CODER_ID = 21;
        public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(115.840);
        public static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(3, DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
    }

    public static final boolean ANGLE_INVERT = true;
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
                MAX_SPEED,
                DRIVE_BASE_RADIUS.in(Meters),
                new ReplanningConfig(true, true));
    }
}
