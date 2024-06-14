package frc.robot.subsystems.swerve;


import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.TunableNumber;

public class SwerveConstants {
    static final int NUMBER_OF_MODULES = 4;

    static final TunableNumber
            AZIMUTH_CONTROLLER_KP = new TunableNumber("Swerve/Yaw (Azimuth of ROBOT) kP", 3),
            AZIMUTH_CONTROLLER_TOLERANCE_DEG = new TunableNumber("Swerve/Yaw (Azimuth of ROBOT) Tolerance", 1),
            AZIMUTH_MAX_VELOCITY = new TunableNumber("Swerve/Yaw (Azimuth of ROBOT) Max Velocity [DEG PS]", 4 * Math.PI),
            AZIMUTH_MAX_ACCELERATION = new TunableNumber("Swerve/Yaw (Azimuth of ROBOT) Max Acceleration [DEG PS]", 4 * Math.PI);

    static final double
            AZIMUTH_CONTROLLER_I = 0,
            AZIMUTH_CONTROLLER_D = 0,
            AZIMUTH_CONTROLLER_DEADBAND = 0.1,
            TRANSLATION_CONTROLLER_P = 1.366,
            TRANSLATION_MAX_VELOCITY = 2.7,
            TRANSLATION_MAX_ACCELERATION = 3;

    static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    static final COTSFalconSwerveConstants CHOSEN_MODULE =
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    static final double WHEEL_BASE = 0.565;
    static final double TRACK_WIDTH = 0.615;

    public static final double DRIVE_BASE_RADIUS = new Translation2d(TRACK_WIDTH / 2, WHEEL_BASE / 2).getNorm();
    public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

    /* SwerveSubsystem Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

    /* Module Gear Ratios */
    static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
    static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

    /* Motor Inverts */
    static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
    static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive;

    /* Angle Encoder Invert */
    static final SensorDirectionValue CAN_CODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;// CHOSEN_MODULE.canCoderInvert;

    /* SwerveSubsystem Current Limiting */
    static final int ANGLE_CURRENT_LIMIT = 30;
    static final int DRIVE_SUPPLY_CURRENT_LIMIT = 35;
    static final int DRIVE_STATOR_CURRENT_LIMIT = 50;

    //These two determine for how long and what current needs to be applied in order to be ramped down.
    static final int DRIVE_PEAK_CURRENT_LIMIT = 40;
    static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;

    static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    static final double OPEN_LOOP_RAMP = 0.1;
    static final double CLOSED_LOOP_RAMP = 0.0;

    /* Angle Motor PID Values */
    static final TunableNumber ANGLE_KP = new TunableNumber("Swerve/Angle MOTOR(Azimuth of WHEEL) kP", 2.5);
    static final double ANGLE_KI = 0;
    static final double ANGLE_KD = 0;

    /* Drive Motor PID Values */
    static final double
            DRIVE_KP = 0.053067,
            DRIVE_KI = 0.0,
            DRIVE_KD = 0.0,

    /* Drive Motor Characterization Values */
    // Convert the values calculated by SysId from volts to [-1, 1], which TalonFX uses.
    DRIVE_KS = 0.27053 / 12.d,
            DRIVE_KV = 0.10861 / 12.d,
            DRIVE_KA = 0.023132 / 12.d;

    /**
     * Meters per Second
     */
    public static final double MAX_SPEED_MPS = 5.1;
    // MAX_ANGULAR_VELOCITY / DRIVE_BASE_RADIUS; //This gives like 22Mps lmfao no it's not true

    /* Neutral Modes */
    static final com.revrobotics.CANSparkBase.IdleMode ANGLE_NEUTRAL_MODE = CANSparkBase.IdleMode.kBrake;
    static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

    /* Module Specific Constants */

    /* Front Left Module - Module 0 */
    static final class Module0 {
        static final int DRIVE_MOTOR_ID = 14;
        static final int ANGLE_MOTOR_ID = 11;
        static final int CAN_CODER_ID = 18;
        static final double ROTATIONS_OFFSET = 0.677246;
        static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(0, DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ROTATIONS_OFFSET);
    }

    /* Front Right Module - Module 1 */
    static final class Module1 {
        static final int DRIVE_MOTOR_ID = 3;
        static final int ANGLE_MOTOR_ID = 10;
        static final int CAN_CODER_ID = 20;
        static final double ROTATIONS_OFFSET = 0.282715;
        static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(1, DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ROTATIONS_OFFSET);
    }

    /* Back Left Module - Module 2 */
    static final class Module2 {
        static final int DRIVE_MOTOR_ID = 13;
        static final int ANGLE_MOTOR_ID = 6;
        static final int CAN_CODER_ID = 19;//
        static final double ROTATIONS_OFFSET = 0.533447;
        static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(2, DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ROTATIONS_OFFSET);
    }

    /* Back Right Module - Module 3 */
    static final class Module3 {
        static final int DRIVE_MOTOR_ID = 2;
        static final int ANGLE_MOTOR_ID = 9;
        static final int CAN_CODER_ID = 21;
        static final double ROTATIONS_OFFSET = 0.313721;
        static final SwerveModuleConstants CONSTANTS =
                new SwerveModuleConstants(3, DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ROTATIONS_OFFSET);
    }

    /**
     * We avoid steering the swerve wheels below a certain driving speed, for in-place turning
     * causes them to jitter. Thus, we hereby define the maximum driving speed that is
     * considered 'in-place'.
     */
    static final double SWERVE_IN_PLACE_DRIVE_MPS = 0.01 * MAX_SPEED_MPS;

    public static final class AutoConstants {
        static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
                new PIDConstants(TRANSLATION_CONTROLLER_P, 0.0, 0.0), // Translation PID constants
                new PIDConstants(AZIMUTH_CONTROLLER_KP.get(), AZIMUTH_CONTROLLER_I, AZIMUTH_CONTROLLER_D), // Rotation PID constants
                MAX_SPEED_MPS,
                DRIVE_BASE_RADIUS,
                new ReplanningConfig(true, true));
    }

    public record SwerveModuleConstants(int moduleNumber, int driveMotorID, int steerMotorID, int canCoderID,
                                        double angleOffset) {
        /**
         * SwerveSubsystem Module Constants to be used when creating swerve modules.
         *
         * @param moduleNumber - Number of the swerve module
         * @param driveMotorID - ID of the drive motor
         * @param steerMotorID - ID of the angle motor
         * @param canCoderID   - ID of the canCoder
         * @param angleOffset  - Offset of canCoder to zero
         */
        public SwerveModuleConstants {
        }
    }
}
