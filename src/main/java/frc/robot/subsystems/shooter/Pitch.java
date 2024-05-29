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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CANSparkMaxUtil;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.math.Conversions.RPMToRotationsPerSecond;
import static frc.lib.util.CTREUtil.applyConfig;
import static frc.robot.Constants.CanIDConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.CanIDConstants.PIVOT_ID;
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
import static frc.robot.subsystems.shooter.ShooterConstants.TIME_DIFFERENCE;

public class Pitch {
    private final CANSparkFlex motor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANcoder absoluteEncoder = new CANcoder(PIVOT_CAN_CODER);
    private final RelativeEncoder encoder;

    private final PIDController controller = new PIDController(PITCH_KP, PITCH_KI, PITCH_KD);
    private final ArmFeedforward feedforward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KA);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(PITCH_MAX_VELOCITY, PITCH_MAX_ACCELERATION);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

    /**
     * The position is in rotations
     * The velocity is in rotations per second.
     */
    private StatusSignal<Double> encoderPositionSignal, encoderVelocitySignal;

    private TrapezoidProfile.State state;
    private TrapezoidProfile.State goal;

    public Pitch() {
        configurePitchMotor();
        configureExternalEncoder();
        configureController();

        encoder = motor.getEncoder();
        state = new TrapezoidProfile.State(getPosition().getRotations(), 0);
    }

    /**
     * This method should be called periodically for the pitch to maintain its position. <br>
     * If no goal is set, this method will do nothing.
     */
    public void periodic() {
        if(goal != null) {
            drivePitchPeriodic();
            logPitch();
        }
    }

    /**
     * Get the commanded goal position, in {@link Rotation2d} <p>
     * If no goal is set, return 0 degrees.
     * @return the commanded position
     */
    public final Rotation2d getGoalPosition() {
        if(goal != null)
            return Rotation2d.fromRotations(goal.position);

        return Rotation2d.fromDegrees(0);
    }

    /**
     * Set the target position goal, from {@link Rotation2d}
     * Target velocity defaults to 0.
     * @param targetPosition The goal position
     */
    public void setGoal(Rotation2d targetPosition) {
        setGoal(new TrapezoidProfile.State(targetPosition.getRotations(), 0));
    }

    /**
     * Returns whether the pitch is at the given goal. If no goal is set, defaults to false.
     * @return whether the pitch is at its goal position.
     */
    public boolean isAtGoal() {
        if(goal == null) return false;

        SmartDashboard.putNumber("pitch/Distance From Goal [ROT]", getPosition().getRotations() - goal.position);
        SmartDashboard.putNumber("pitch/Distance to Goal Tolerance [ROT]", PITCH_TOLERANCE);

        return Math.abs(getPosition().getRotations() - goal.position) < PITCH_TOLERANCE;
    }


    /**
     * Returns the current position of the pitch. This is measured considering the horizontal as 0.
     * @return The current position of the pitch.
     */
    public Rotation2d getPosition() {
        Rotation2d angle = Rotation2d.fromRotations(encoderPositionSignal.refresh().getValue()).plus(PIVOT_ENCODER_OFFSET);

        //Ensure the encoder always has the same starting angle.
        if (angle.getDegrees() > 300)
            angle.minus(Rotation2d.fromDegrees(360));

        return angle;
    }

    /**
     * Return the velocity of the Pitch. Uses the Relative encoder for reduced latency.
     * @Units Rotations per second
     * @return the velocity
     */
    public double getVelocity() {
        return RPMToRotationsPerSecond(encoder.getVelocity(), 1.0/150);
    }

    /**
     * Returns the voltage the motor
     * @Units Volts
     * @return the amount of voltage the motor uses
     */
    public Measure<Voltage> getVoltage() {
        return Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    }

    /**
     * Give the motor a voltage amount. Can be between -12 and 12.
     * @Units Volts
     * @param voltage the amount of voltage to give the motor
     */
    public void drivePitch(double voltage) {
        motor.setVoltage(voltage);
    }

    private void configureExternalEncoder() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        applyConfig(absoluteEncoder, canCoderConfig);

        encoderPositionSignal = absoluteEncoder.getPosition().clone();
        encoderVelocitySignal = absoluteEncoder.getVelocity().clone();

        encoderPositionSignal.setUpdateFrequency(50);
        encoderVelocitySignal.setUpdateFrequency(50);

        absoluteEncoder.optimizeBusUtilization();
    }

    private void configurePitchMotor() {
        motor.restoreFactoryDefaults();

        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.setSmartCurrentLimit(60);
        motor.enableVoltageCompensation(12);

        CANSparkMaxUtil.setCANSparkBusUsage(motor, CANSparkMaxUtil.Usage.kMinimal);

        motor.burnFlash();
    }

    private void logPitch() {
        SmartDashboard.putNumber("pitch/currentAbsolutePosition", getPosition().getDegrees());
        SmartDashboard.putNumber("pitch/velocity [ DEG ]", getVelocity() * 360);
        SmartDashboard.putNumber("pitch/goalPosition [DEG]", Rotation2d.fromRotations(goal.position).getDegrees());
        SmartDashboard.putNumber("pitch/goalVelocity [RotPS]", goal.velocity);
        SmartDashboard.putBoolean("pitch/isAtGoal", isAtGoal());
        SmartDashboard.putNumber("pitch/ElectricityCurrent [A]", motor.getOutputCurrent());
        SmartDashboard.putNumber("pitch/ElectricityVoltage [V]", getVoltage().in(Volts));
    }

    private void configureController() {
        controller.setTolerance(PITCH_TOLERANCE);
        controller.reset();
    }

    private void drivePitchPeriodic() {
        state = profile.calculate(TIME_DIFFERENCE, state, goal);
        drivePitchToSetpoint(state);
    }

    private void drivePitchToSetpoint(TrapezoidProfile.State setpoint) {
        double feedforwardOutput = feedforward.calculate(
                Units.rotationsToRadians(setpoint.position),
                setpoint.velocity
        );

        double controllerOutput = controller.calculate(
                getPosition().getRotations(),
                goal.position
        );

        double voltageOutput = applyDeadband(feedforwardOutput + controllerOutput, 0.02);

        motor.setVoltage(voltageOutput);
    }

    private void setGoal(TrapezoidProfile.State goal) {
        controller.reset();
        state = new TrapezoidProfile.State(getPosition().getRotations(), 0);

        this.goal = goal;
    }
}