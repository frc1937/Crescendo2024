package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
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
import static frc.robot.subsystems.shooter.ShooterConstants.PIVOT_TOLERANCE;
import static frc.robot.subsystems.shooter.ShooterConstants.TIME_DIFFERENCE;

public class Pitch {
    private final CANSparkFlex motor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANcoder absoluteEncoder = new CANcoder(PIVOT_CAN_CODER);

    private PIDController controller;

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

        state = new TrapezoidProfile.State(getPosition().getRotations(), 0);
        setGoal(getPosition());
    }

    public void periodic() {
        drivePitchPeriodic();
        logPitch();
    }

    public void drivePitchToSetpoint(TrapezoidProfile.State setpoint) {
        //I assume the angle is needed in radians.
        double feedforwardOutput = applyDeadband(feedforward.calculate(
                Units.rotationsToRadians(setpoint.position),
                setpoint.velocity
                //this is presumably in the correct units? tried to convert to RadPS and it smashed into the floor.
        ), 0.02);

        double controllerOutput = applyDeadband(controller.calculate(
                getPosition().getRotations(),
                setpoint.position
        ), 0.02);

        motor.setVoltage(feedforwardOutput + controllerOutput);
    }

    public final Rotation2d getGoalPosition() {
        return Rotation2d.fromRotations(goal.position);
    }

    public final void setGoal(Rotation2d targetPosition) {
        setGoal(new TrapezoidProfile.State(targetPosition.getRotations(), 0));
    }

    public final void setGoal(TrapezoidProfile.State goal) {
        controller.reset();
        this.goal = goal;
    }

    public boolean isAtGoal() {
        return Math.abs(getPosition().getRotations() - goal.position) < PIVOT_TOLERANCE;
    }

    public Rotation2d getPosition() {
        Rotation2d angle = Rotation2d.fromRotations(encoderPositionSignal.refresh().getValue()).minus(PIVOT_ENCODER_OFFSET);

        //Ensure the encoder always has the same starting angle.
        if (angle.getDegrees() > 300)
            angle.minus(Rotation2d.fromDegrees(360));

        return angle;
    }

    public double getVelocity() {
        return encoderVelocitySignal.refresh().getValue();
    }

    public Measure<Voltage> getVoltage() {
        return Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    }

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
        SmartDashboard.putNumber("pitch/goalPosition [DEG]", Rotation2d.fromRotations(goal.position).getDegrees());
        SmartDashboard.putNumber("pitch/goalVelocity [RotPS]", goal.velocity);
        SmartDashboard.putBoolean("pitch/isAtGoal", isAtGoal());
        SmartDashboard.putNumber("pitch/ElectricityCurrent [A]", motor.getOutputCurrent());
        SmartDashboard.putNumber("pitch/ElectricityVoltage [V]", getVoltage().in(Volts));
    }

    private void configureController() {
        controller = new PIDController(PITCH_KP, PITCH_KI, PITCH_KD);
        controller.setTolerance(PITCH_TOLERANCE);
    }

    private void drivePitchPeriodic() {
        state = profile.calculate(TIME_DIFFERENCE, state, goal);
        drivePitchToSetpoint(state);
    }
}