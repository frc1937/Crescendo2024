package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.generic.Properties;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.encoder.EncoderConfiguration;
import frc.lib.generic.encoder.EncoderProperties;
import frc.lib.generic.encoder.GenericCanCoder;
import frc.lib.generic.motor.GenericSpark;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.GlobalConstants.CanIDConstants.PIVOT_CAN_CODER;
import static frc.robot.GlobalConstants.CanIDConstants.PIVOT_ID;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_DEFAULT_ANGLE;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_GEAR_RATIO;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KA;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KD;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KG;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KI;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KP;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KS;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KV;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_MAX_ACCELERATION;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_MAX_VELOCITY;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_TOLERANCE;
import static frc.robot.subsystems.shooter.ShooterConstants.PIVOT_ENCODER_OFFSET;

public class Pitch {
    private final Motor motor = new GenericSpark(PIVOT_ID, MotorProperties.SparkType.FLEX);
    private final Encoder absoluteEncoder = new GenericCanCoder(PIVOT_CAN_CODER);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(PITCH_MAX_VELOCITY, PITCH_MAX_ACCELERATION);
    private final ProfiledPIDController feedback = new ProfiledPIDController(PITCH_KP, PITCH_KI, PITCH_KD, constraints);
    private final ArmFeedforward feedforward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KA);

    private TrapezoidProfile.State goal;

    public Pitch() {
        configurePitchMotor();
        configureExternalEncoder();
        configureController();
    }

    /**
     * This method should be called periodically for the pitch to maintain its position. <br>
     * If no goal is set, this method will do nothing.
     */
    public void periodic() {
        if (goal != null) {
            drivePitchToSetpoint();
            logPitch();
        }
    }

    /**
     * Get the commanded goal position, in {@link Rotation2d} <p>
     * If no goal is set, return 0 degrees.
     *
     * @return the commanded position
     */
    public final Rotation2d getGoalPosition() {
        if (goal != null)
            return Rotation2d.fromRotations(goal.position);

        return PITCH_DEFAULT_ANGLE;
    }

    /**
     * Set the target position goal, from {@link Rotation2d}
     * Target velocity defaults to 0.
     *
     * @param targetPosition The goal position
     */
    public void setGoal(Rotation2d targetPosition) {
        setGoal(new TrapezoidProfile.State(targetPosition.getRotations(), 0));
    }

    /**
     * Returns whether the pitch is at the given goal. If no goal is set, defaults to false.
     *
     * @return whether the pitch is at its goal position.
     */
    public boolean isAtGoal() {
        if (goal == null) return false;

        SmartDashboard.putNumber("pitch/Distance From Goal [ROT]", getPosition().getRotations() - goal.position);
        SmartDashboard.putNumber("pitch/Distance to Goal Tolerance [ROT]", PITCH_TOLERANCE);

        return Math.abs(getPosition().getRotations() - goal.position) < PITCH_TOLERANCE;
    }

    /**
     * Returns the current position of the pitch. This is measured considering the horizontal as 0.
     *
     * @return The current position of the pitch.
     */
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(absoluteEncoder.getEncoderPosition());
    }

    /**
     * Return the velocity of the Pitch. Uses the Relative encoder for reduced latency.
     *
     * @return the velocity
     * @Units Rotations per second
     */
    public double getVelocity() {
        return motor.getSystemVelocity() / 60;
    }

    public void setIdleMode(MotorProperties.IdleMode idleMode) {
        motor.setIdleMode(idleMode);
    }

    /**
     * Returns the voltage the motor
     *
     * @return the amount of voltage the motor uses
     * @Units Volts
     */
    public Measure<Voltage> getVoltage() {
        return Volts.of(motor.getVoltage() * 12);
    }

    /**
     * Returns the relative encoder's position
     *
     * @return the position of the relative encoder
     * @Units Rotations of the motor
     */
    public double getRelativeEncoderPosition() {
        return motor.getSystemPosition();
    }

    /**
     * Give the motor a voltage amount. Can be between -12 and 12.
     *
     * @param voltage the amount of voltage to give the motor
     * @Units Volts
     */
    public void setRawVoltage(double voltage) {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    public void resetController() {
        feedback.reset(getPosition().getRotations(), getVelocity());
    }

    private void logPitch() {
        SmartDashboard.putNumber("pitch/currentAbsolutePosition", getPosition().getDegrees());
        SmartDashboard.putNumber("pitch/CurrentVelocity [RotPS]", getVelocity() * 360);
        SmartDashboard.putNumber("pitch/goalPosition [DEG]", Rotation2d.fromRotations(goal.position).getDegrees());
        SmartDashboard.putNumber("pitch/goalVelocity [RotPS]", goal.velocity);
        SmartDashboard.putBoolean("pitch/isAtGoal", isAtGoal());
        SmartDashboard.putNumber("pitch/ElectricityCurrent [A]", motor.getCurrent());
        SmartDashboard.putNumber("pitch/ElectricityVoltage [V]", getVoltage().in(Volts));
    }

    private void configureController() {
        feedback.setTolerance(PITCH_TOLERANCE);
    }

    private void drivePitchToSetpoint() {
        final double controllerOutput = feedback.calculate(getPosition().getRotations());

        final double feedforwardOutput = feedforward.calculate(
                Units.rotationsToRadians(feedback.getSetpoint().position),
                feedback.getSetpoint().velocity
        );

        final double voltageOutput = feedforwardOutput + controllerOutput;

        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltageOutput);
    }

    private void setGoal(TrapezoidProfile.State goal) {
        feedback.setGoal(goal);
        this.goal = goal;
    }

    private void configurePitchMotor() {
        MotorConfiguration configuration = new MotorConfiguration();

        configuration.conversionFactor = PITCH_GEAR_RATIO;

        configuration.idleMode = MotorProperties.IdleMode.BRAKE;
        configuration.supplyCurrentLimit = 40;

        motor.setMotorPosition(getPosition().getRotations());
    }

    private void configureExternalEncoder() {
        EncoderConfiguration encoderConfiguration = new EncoderConfiguration();

        encoderConfiguration.invert = true;
        encoderConfiguration.offsetRotations = PIVOT_ENCODER_OFFSET.getRotations();
        encoderConfiguration.sensorRange = EncoderProperties.SensorRange.NegativeHalfToHalf;

        absoluteEncoder.configure(encoderConfiguration);

        absoluteEncoder.setSignalUpdateFrequency(Properties.SignalType.POSITION, 50);
    }
}