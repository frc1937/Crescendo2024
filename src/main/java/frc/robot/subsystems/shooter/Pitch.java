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
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.util.CTREUtil.applyConfig;
import static frc.robot.Constants.CanIDConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.CanIDConstants.PIVOT_ID;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KA;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KG;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KS;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KV;
import static frc.robot.subsystems.shooter.ShooterConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.subsystems.shooter.ShooterConstants.PIVOT_TOLERANCE;

public class Pitch {
    private static final double TIME_DIFFERENCE = 0.02;

    private TrapezoidProfile.State state;
    private TrapezoidProfile.State goal;

    private final CANSparkFlex motor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANcoder absoluteEncoder = new CANcoder(PIVOT_CAN_CODER);

    private final RelativeEncoder relativeEncoder;
    private final SparkPIDController controller;

    private final ArmFeedforward feedforward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KA);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(4, 6);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

    private StatusSignal<Double> encoderPositionSignal, encoderVelocitySignal;

    public Pitch() {
        configurePitchMotor();
        configureSteerEncoder();
        configureSoftLimits();

        controller = motor.getPIDController();
        controller.setP(1);
        controller.setI(0);
        controller.setD(0);
        controller.setOutputRange(-12, 12);

        relativeEncoder = motor.getEncoder();
        relativeEncoder.setPosition(getPosition().getRotations());

        state = new TrapezoidProfile.State(getPosition().getRotations(), 0);
        setGoal(getPosition());
    }

    public void periodic() {
        state = profile.calculate(TIME_DIFFERENCE, state, goal);
        drivePitchToSetpoint(state);

        SmartDashboard.putNumber("pitch/currentRelativePosition", Rotation2d.fromRotations(relativeEncoder.getPosition()).getDegrees());
        SmartDashboard.putNumber("pitch/currentAbsolutePosition", getPosition().getDegrees());
        SmartDashboard.putNumber("pitch/goalPosition [DEG]", Rotation2d.fromRotations(goal.position).getDegrees());
        SmartDashboard.putNumber("pitch/goalVelocity [RPS]", goal.velocity);
        SmartDashboard.putBoolean("pitch/isAtGoal", isAtGoal());
        SmartDashboard.putNumber("pitch/ElectricityCurrent [A]", motor.getOutputCurrent());
        SmartDashboard.putNumber("pitch/ElectricityVoltage [V]", getVoltage().in(Volts));

        relativeEncoder.setPosition(getPosition().getRotations());
    }

    public void drivePitchToSetpoint(TrapezoidProfile.State setpoint) {
        double feedforwardOutput = applyDeadband(feedforward.calculate(setpoint.position, setpoint.velocity), 0.02);

        controller.setReference(
                setpoint.position,
                CANSparkBase.ControlType.kPosition,
                0,
                feedforwardOutput
        );
    }

    public final Rotation2d getGoalPosition() {
        return Rotation2d.fromRotations(goal.position);
    }

    public final void setGoal(Rotation2d targetPosition) {
        setGoal(new TrapezoidProfile.State(targetPosition.getRotations(), 0));
    }

    public final void setGoal(TrapezoidProfile.State goal) {
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

    private void configureSteerEncoder() {
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
        motor.setSmartCurrentLimit(40);
        motor.burnFlash();
    }

    private void configureSoftLimits() {
        motor.getEncoder().setPosition(getPosition().getRotations());
    }
}

/*

 */