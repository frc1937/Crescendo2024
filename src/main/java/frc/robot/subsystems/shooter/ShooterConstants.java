package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.*;

public class ShooterConstants {
    static final Rotation2d FORWARD_PITCH_SOFT_LIMIT = Rotation2d.fromDegrees(10);
    static final Rotation2d REVERSE_PITCH_SOFT_LIMIT = Rotation2d.fromDegrees(-15);

    public static final int FLYWHEEL_MAX_RPM = 6400;
    public static final double PITCH_INTAKE_FEEDER_ANGLE = 51;
    public static final double POSE_HISTORY_DURATION = 0.3;

    static final double DEFAULT_SLOPE_TO_VIRTUAL_TARGET = 0.5;
    static final double CONSIDERED_NOISELESS_THRESHOLD = 20;

    static final Rotation2d PITCH_DEFAULT_ANGLE = Rotation2d.fromDegrees(0);
    static final Rotation2d PIVOT_ENCODER_OFFSET = Rotation2d.fromDegrees(-24.785156);
    static final Rotation2d PIVOT_BOTTOM_ANGLE = Rotation2d.fromDegrees(-16.787110);

    static final double
            PITCH_KS = 0.38398,
            PITCH_KG = 0.31481,
            PITCH_KV = 12.824,
            PITCH_KA = 3.995,
            PITCH_KP = 1,//1.d / 0.02,
            PITCH_KI = 0.0,
            PITCH_KD = 0.0,
            PITCH_MAX_VELOCITY = 25,//1.05,
            PITCH_MAX_ACCELERATION = 35;//0.75;

    static final float PIVOT_CONSTRAINT_DEGREES = 130;  // TODO This is not the final value
    static final CANSparkBase.SoftLimitDirection PIVOT_CONSTRAINT_DIRECTION = CANSparkBase.SoftLimitDirection.kForward;

    static final double PIVOT_TOLERANCE = Units.degreesToRadians(1);
    static final float PITCH_TRANSMISSION_RATIO = 150;
    static final double DEFAULT_PITCH_DEADBAND = 0.025, VERTICAL_PITCH_DEADBAND = 0.06;

    static final Measure<Distance> LEFT_FLYWHEEL_DIAMETER = Inches.of(4);
    static final Measure<Distance> RIGHT_FLYWHEEL_DIAMETER = Inches.of(3);

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

    public static final double KICKER_SPEED_BACKWARDS = -0.7;
    public static final double KICKER_SPEED_FORWARD = 1;

    public static final ShooterSubsystem.Reference SPEAKER_FRONT = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(50), MetersPerSecond.of(16));
    public static final ShooterSubsystem.Reference SPEAKER_BACK = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(111.5), MetersPerSecond.of(14));
    public static final ShooterSubsystem.Reference ASSIST = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(48), MetersPerSecond.of(15));
    public static final ShooterSubsystem.Reference INTAKE = new ShooterSubsystem.Reference(
            PIVOT_BOTTOM_ANGLE.plus(Rotation2d.fromDegrees(1)), MetersPerSecond.of(-3));

    public static final ShooterSubsystem.Reference AMP_INIT =
            new ShooterSubsystem.Reference(Rotation2d.fromDegrees(102), MetersPerSecond.of(3));
}
