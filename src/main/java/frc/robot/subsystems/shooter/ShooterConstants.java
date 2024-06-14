package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.*;

public class ShooterConstants {
    static final Rotation2d FORWARD_PITCH_SOFT_LIMIT = Rotation2d.fromDegrees(10);
    static final Rotation2d REVERSE_PITCH_SOFT_LIMIT = Rotation2d.fromDegrees(-15);

    public static final int FLYWHEEL_MAX_RPM = 5500;

    static final double CONSIDERED_NOISELESS_THRESHOLD = 20;
    static final double GRAVITY_FORCE = 9.8;

    static final double PITCH_GEAR_RATIO = 1.0 / 149;

    static final Rotation2d PITCH_DEFAULT_ANGLE = Rotation2d.fromDegrees(20);
    static final Rotation2d PIVOT_ENCODER_OFFSET = Rotation2d.fromDegrees(21.478516);
    static final Rotation2d PIVOT_BOTTOM_ANGLE = Rotation2d.fromDegrees(-18.984374625);

    static final double
            PITCH_KS = 0,
            PITCH_KV = 14,
            PITCH_KA = 1.2632,
            PITCH_KG = 0.23,

            PITCH_KP = 40,
            PITCH_KI = 0.0,
            PITCH_KD = 0.0,

            PITCH_MAX_VELOCITY = 0.5,
            PITCH_MAX_ACCELERATION = 0.5;

    static final double PITCH_TOLERANCE = Rotation2d.fromDegrees(0.3).getRotations();
    static final Measure<Velocity<Angle>> FLYWHEEL_TOLERANCE = RPM.of(70);

    static final double LEFT_FLYWHEEL_DIAMETER = Inches.of(3).in(Meters);
    static final double RIGHT_FLYWHEEL_DIAMETER = Inches.of(4).in(Meters);

    /**
     * All in rotations per second and voltages
     */
    static final double
            FLYWHEEL_RIGHT_P = 0.003,//0.027796,
            FLYWHEEL_RIGHT_S = 0.022648,
            FLYWHEEL_RIGHT_V = 0.097728,

            FLYWHEEL_LEFT_P = 0.003,//0.03,  // NOTE: This value was not provided by SysId
            FLYWHEEL_LEFT_S = 0.02426,
            FLYWHEEL_LEFT_V = 0.0969743;

    static final double PIVOT_POINT_Z_OFFSET_METRES = 0.29;
    static final double PIVOT_POINT_X_OFFSET_METRES = -0.27;
    static final double SHOOTER_LENGTH_METRES = 0.5;

    public static final double KICKER_SPEED_BACKWARDS = -0.6;
    public static final double KICKER_SPEED_FORWARD = 0.5;

    //Some presets.
    public static final ShooterSubsystem.Reference SPEAKER_FRONT = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(50), 16);

    public static final ShooterSubsystem.Reference SPEAKER_BACK = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(111.5), 14);
    public static final ShooterSubsystem.Reference ASSIST = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(48), 15);
    public static final ShooterSubsystem.Reference INTAKE = new ShooterSubsystem.Reference(
            PIVOT_BOTTOM_ANGLE, -10);
    public static final ShooterSubsystem.Reference AMP_INIT =
            new ShooterSubsystem.Reference(Rotation2d.fromDegrees(102), 3);
}
