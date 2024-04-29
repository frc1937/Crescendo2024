package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.Target;

import static edu.wpi.first.units.Units.RPM;

public class ShooterConstants {
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
    public static final int FLYWHEEL_MAX_RPM = 6400;
    public static final double PITCH_INTAKE_FEEDER_ANGLE = 51;
    public static final double POSE_HISTORY_DURATION = 0.3;

    static final double DEFAULT_SLOPE_TO_VIRTUAL_TARGET = 0.5;
    static final double PITCH_INTAKE_FLOOR_ANGLE = -21.583960;
    static final double CONSIDERED_NOISELESS_THRESHOLD = 20;
    static final Rotation2d PITCH_DEFAULT_ANGLE = Rotation2d.fromDegrees(PITCH_INTAKE_FLOOR_ANGLE);

    static final int
            FLYWHEEL_LEFT_ID = 16,
            FLYWHEEL_RIGHT_ID = 15,
            PIVOT_ID = 1,
            PIVOT_CAN_CODER = 22,
            KICKER_ID = 8;
    static final double PIVOT_ENCODER_OFFSET = 343,
            PITCH_KS = 0.38398,
            PITCH_KG = 0.31481,
            PITCH_KV = 12.824,
            PITCH_KA = 3.995,
            PITCH_KP = 1.5,//1.d / 0.02,
            PITCH_KD = 0.0,
            PITCH_MAX_VELOCITY = 25,//1.05,
            PITCH_MAX_ACCELERATION = 35;//0.75;
    static final float PIVOT_CONSTRAINT_DEGREES = 130;  // TODO This is not the final value
    static final double PIVOT_TOLERANCE = Units.degreesToRadians(1);
    static final CANSparkBase.SoftLimitDirection PIVOT_CONSTRAINT_DIRECTION = CANSparkBase.SoftLimitDirection.kForward;
    static final float PITCH_TRANSMISSION_RATIO = 150;
    static final double DEFAULT_PITCH_DEADBAND = 0.025, VERTICAL_PITCH_DEADBAND = 0.06;

    static final class FlywheelControlConstants {
        /**
         * All in rotations per second and voltages
         */
        static final double
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

    public static final Target SPEAKER_TARGET = new Target(new Translation2d(0.03, 5.3));
    public static final Target ASSIST_TARGET = new Target(new Translation2d(-0.5, 8.0));
    public static final double FIELD_LENGTH = 16.48;

    static {
        SPEAKER_TARGET.putMeasurement(1.24, 53, 2500, 1);
        SPEAKER_TARGET.putMeasurement(1.56, 47, 2500, 1);
        SPEAKER_TARGET.putMeasurement(1.85, 44, 2500, 1);
        SPEAKER_TARGET.putMeasurement(2.08, 40, 2700, 1);
        SPEAKER_TARGET.putMeasurement(2.33, 35, 3000, 0.85);
        SPEAKER_TARGET.putMeasurement(2.72, 34, 3000, 0.85);
        SPEAKER_TARGET.putMeasurement(2.95, 31, 3500, 0.85);
        SPEAKER_TARGET.putMeasurement(3.27, 27.8, 3500, 0.85);
        SPEAKER_TARGET.putMeasurement(3.7, 26.1, 4000, 0.8);
        SPEAKER_TARGET.putMeasurement(4.1, 22.9, 5000, 0.9);
        SPEAKER_TARGET.putMeasurement(4.35, 21.8, 5400, 0.9);
        SPEAKER_TARGET.putMeasurement(4.7, 21.5, 5500, 0.9);
        SPEAKER_TARGET.putMeasurement(4.82, 20.235, 6150, 1.1);

        SPEAKER_TARGET.putTimeOfFlightMeasurement(1.d, 0.2);
        SPEAKER_TARGET.putTimeOfFlightMeasurement(5.d, 0.4);

        SPEAKER_TARGET.putAzimuthToleranceMeasurement(2.d, 3.8);
        SPEAKER_TARGET.putAzimuthToleranceMeasurement(4.d, 3.0);
        SPEAKER_TARGET.putAzimuthToleranceMeasurement(7.d, 1.4);
    }

    static {
        ASSIST_TARGET.putMeasurement(5, 60, 1750, 0.7);
        ASSIST_TARGET.putMeasurement(8, 50, 2350, 0.75);
        ASSIST_TARGET.putMeasurement(11, 36, 2900, 0.8);
        ASSIST_TARGET.putMeasurement(14, 36, 3500, 0.83);
        ASSIST_TARGET.putMeasurement(17, 36, 4100, 0.85);

        ASSIST_TARGET.putTimeOfFlightMeasurement(1.d, 1.5);
        ASSIST_TARGET.putTimeOfFlightMeasurement(17.d, 2.6);

        ASSIST_TARGET.putAzimuthToleranceMeasurement(2.d, 5.d);
        ASSIST_TARGET.putAzimuthToleranceMeasurement(15.d, 3);
    }

    public static final double KICKER_SPEED_BACKWARDS = -0.7;
    public static final double KICKER_SPEED_FORWARD = 1;

    public static final ShooterSubsystem.Reference SPEAKER_FRONT = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(50), RPM.of(2600));//50!
    public static final ShooterSubsystem.Reference SPEAKER_BACK = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(111.5), RPM.of(2500));
    public static final ShooterSubsystem.Reference ASSIST = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(48), RPM.of(2500));//50!

    public static final ShooterSubsystem.Reference INTAKE = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(-21.2), RPM.of(-3000), 0.75, 10);

    public static final ShooterSubsystem.Reference AMP_INIT =
            new ShooterSubsystem.Reference(Rotation2d.fromDegrees(102), RPM.of(500));
}
