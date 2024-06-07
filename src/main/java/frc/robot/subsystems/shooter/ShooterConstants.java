package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.*;

public class ShooterConstants {
    static final Rotation2d FORWARD_PITCH_SOFT_LIMIT = Rotation2d.fromDegrees(10);
    static final Rotation2d REVERSE_PITCH_SOFT_LIMIT = Rotation2d.fromDegrees(-15);

    public static final int FLYWHEEL_MAX_RPM = 5400;

    static final double CONSIDERED_NOISELESS_THRESHOLD = 20;
    static final double GRAVITY_FORCE = 9.8;

    static final double PITCH_GEAR_RATIO = 1.0 / 149;

    static final Rotation2d PITCH_DEFAULT_ANGLE = Rotation2d.fromDegrees(0);
    static final Rotation2d PIVOT_ENCODER_OFFSET = Rotation2d.fromDegrees(21.478516);
    static final Rotation2d PIVOT_BOTTOM_ANGLE = Rotation2d.fromDegrees(-18.984374625);

    static final double
            PITCH_KS = 0.16255,
            PITCH_KV = 15.528,
            PITCH_KA = 0.41253,
            PITCH_KG = 0.17103,

            PITCH_KP = 50.0,
            PITCH_KI = 0.0,
            PITCH_KD = 0.0,

            PITCH_MAX_VELOCITY = 2.2,
            PITCH_MAX_ACCELERATION = 0.5;

    static final double PITCH_TOLERANCE = Rotation2d.fromDegrees(0.3).getRotations();
    static final Measure<Velocity<Angle>> FLYWHEEL_TOLERANCE = RPM.of(70);

    static final Measure<Distance> LEFT_FLYWHEEL_DIAMETER = Inches.of(3);
    static final Measure<Distance> RIGHT_FLYWHEEL_DIAMETER = Inches.of(4);

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

    static final double PIVOT_POINT_Z_OFFSET_METRES = 0.21;
    static final double PIVOT_POINT_X_OFFSET_METRES = -0.31;
    static final double SHOOTER_LENGTH_METRES = 0.485;

    public static final double KICKER_SPEED_BACKWARDS = -0.6;
    public static final double KICKER_SPEED_FORWARD = 0.9;

    //Some presets.
    public static final ShooterSubsystem.Reference SPEAKER_FRONT = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(50), MetersPerSecond.of(16));

    public static final ShooterSubsystem.Reference SPEAKER_BACK = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(111.5), MetersPerSecond.of(14));
    public static final ShooterSubsystem.Reference ASSIST = new ShooterSubsystem.Reference(
            Rotation2d.fromDegrees(48), MetersPerSecond.of(15));
    public static final ShooterSubsystem.Reference INTAKE = new ShooterSubsystem.Reference(
            PIVOT_BOTTOM_ANGLE, MetersPerSecond.of(-3));
    public static final ShooterSubsystem.Reference AMP_INIT =
            new ShooterSubsystem.Reference(Rotation2d.fromDegrees(102), MetersPerSecond.of(3));
}
