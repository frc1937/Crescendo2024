package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CANSparkMaxUtil;

import static edu.wpi.first.units.Units.Volts;
import static frc.lib.math.Conversions.SEC_PER_MIN;
import static frc.lib.util.CTREUtil.applyConfig;
import static frc.robot.Constants.CanIDConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.CanIDConstants.PIVOT_ID;
import static frc.robot.Constants.VOLTAGE_COMPENSATION_SATURATION;
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
    private final CANSparkFlex motor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANcoder absoluteEncoder = new CANcoder(PIVOT_CAN_CODER);
    private RelativeEncoder encoder;

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(PITCH_MAX_VELOCITY, PITCH_MAX_ACCELERATION);
    private final ProfiledPIDController feedback = new ProfiledPIDController(PITCH_KP, PITCH_KI, PITCH_KD, constraints);
    private final ArmFeedforward feedforward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KA);

    private double previousVelocitySetpoint;

    /**
     * The position is in rotations.
     * The velocity is in rotations per second.
     */
    private StatusSignal<Double> encoderPositionSignal;

    private TrapezoidProfile.State state;
    private TrapezoidProfile.State goal;

    public Pitch() {
        configurePitchMotor();
        configureExternalEncoder();
        configureController();
        configureInternalEncoder();

        state = new TrapezoidProfile.State(getPosition().getRotations(), getVelocity());
    }

    /**
     * This method should be called periodically for the pitch to maintain its position. <br>
     * If no goal is set, this method will do nothing.
     */
    public void periodic() {
//        encoder.setPosition(getPosition().getRotations());
        if (goal != null) {
            drivePitchPeriodic();
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
        return Rotation2d.fromRotations(encoderPositionSignal.refresh().getValue());
    }

    /**
     * Return the velocity of the Pitch. Uses the Relative encoder for reduced latency.
     *
     * @return the velocity
     * @Units Rotations per second
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }


    public void setBrake(boolean shouldBrake) {
        motor.setIdleMode(shouldBrake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    /**
     * Returns the voltage the motor
     *
     * @return the amount of voltage the motor uses
     * @Units Volts
     */
    public Measure<Voltage> getVoltage() {
        return Volts.of(motor.getAppliedOutput() * 12);
    }

    /**
     * Returns the relative encoder's position
     *
     * @return the position of the relative encoder
     * @Units Rotations of the motor
     */
    public double getRelativeEncoderPosition() {
        return encoder.getPosition();
    }

    /**
     * Give the motor a voltage amount. Can be between -12 and 12.
     *
     * @param voltage the amount of voltage to give the motor
     * @Units Volts
     */
    public void setRawVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    private void configureExternalEncoder() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        canCoderConfig.MagnetSensor.MagnetOffset = PIVOT_ENCODER_OFFSET.getRotations();

        applyConfig(absoluteEncoder, canCoderConfig);

        encoderPositionSignal = absoluteEncoder.getPosition().clone();
        encoderPositionSignal.setUpdateFrequency(50);

        absoluteEncoder.optimizeBusUtilization();
    }

    private void logPitch() {
        SmartDashboard.putNumber("pitch/currentAbsolutePosition", getPosition().getDegrees());
        SmartDashboard.putNumber("pitch/CurrentVelocity [RotPS]", getVelocity() * 360);
        SmartDashboard.putNumber("pitch/goalPosition [DEG]", Rotation2d.fromRotations(goal.position).getDegrees());
        SmartDashboard.putNumber("pitch/goalVelocity [RotPS]", goal.velocity);
        SmartDashboard.putBoolean("pitch/isAtGoal", isAtGoal());
        SmartDashboard.putNumber("pitch/ElectricityCurrent [A]", motor.getOutputCurrent());
        SmartDashboard.putNumber("pitch/ElectricityVoltage [V]", getVoltage().in(Volts));
    }

    private void configureController() {
        feedback.setTolerance(PITCH_TOLERANCE);
    }

    private void drivePitchPeriodic() {
        drivePitchToSetpoint();

        previousVelocitySetpoint = state.velocity;
    }



    private void drivePitchToSetpoint() {
        final double controllerOutput = feedback.calculate(
                getPosition().getRotations()
        );
        final double feedforwardOutput = feedforward.calculate(
                Units.rotationsToRadians(feedback.getSetpoint().position),
                feedback.getSetpoint().velocity
//                (setpoint.velocity - previousVelocitySetpoint) / ROBORIO_LOOP_TIME_SECONDS
        );
        SmartDashboard.putNumber("nigggaaa", feedback.getSetpoint().position * 360);

        final double voltageOutput = feedforwardOutput + controllerOutput;

        motor.setVoltage(voltageOutput);
    }

    public void resetController() {
        feedback.reset(getPosition().getRotations(), getVelocity());
    }

    private void setGoal(TrapezoidProfile.State goal) {
        feedback.setGoal(goal);

        previousVelocitySetpoint = getVelocity();
        this.goal = goal;
    }

    private void configureInternalEncoder() {
        encoder = motor.getEncoder();

        encoder.setPositionConversionFactor(PITCH_GEAR_RATIO);
        encoder.setVelocityConversionFactor(PITCH_GEAR_RATIO / SEC_PER_MIN);
        encoder.setPosition(getPosition().getRotations());
    }

    private void configurePitchMotor() {
        motor.restoreFactoryDefaults();

        motor.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);

        CANSparkMaxUtil.setCANSparkBusUsage(motor, CANSparkMaxUtil.Usage.kMinimal);

        motor.burnFlash();
    }
}