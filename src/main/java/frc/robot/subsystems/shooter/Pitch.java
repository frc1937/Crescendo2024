package frc.robot.subsystems.shooter;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;
import static frc.lib.util.CTREUtil.applyConfig;
import static frc.robot.Constants.CanIDConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.CanIDConstants.PIVOT_ID;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class Pitch {
    private final CANSparkFlex motor = new CANSparkFlex(PIVOT_ID, MotorType.kBrushless);
    private final CANcoder absoluteEncoder = new CANcoder(PIVOT_CAN_CODER);
    private final ArmFeedforward feedforward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KA);
    private final ProfiledPIDController controller;

    private double deadband = DEFAULT_PITCH_DEADBAND;
    private StatusSignal<Double> encoderPositionSignal, encoderVelocitySignal;

    public Pitch() {
        // Configure the motor
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        configureSteerEncoder();
        configureSoftLimits();

        controller = new ProfiledPIDController(
                PITCH_KP, 0, PITCH_KD,
                new TrapezoidProfile.Constraints(PITCH_MAX_VELOCITY, PITCH_MAX_ACCELERATION));

        controller.setTolerance(PIVOT_TOLERANCE);
    }

    public void periodic() {
        SmartDashboard.putNumber("pitch/CurrentAngle", getCurrentPosition().getDegrees());
        SmartDashboard.putNumber("pitch/Goal", Units.radiansToDegrees(controller.getGoal().position));
        SmartDashboard.putNumber("pitch/VelocitySetpoint", Units.radiansToDegrees(controller.getSetpoint().velocity));

//        double velocitySetpoint = MathUtil.applyDeadband(
//                controller.calculate(getCurrentPosition().getRadians()), deadband);
//
//        double voltage = feedforward.calculate(getCurrentPosition().getRadians(), velocitySetpoint);
//        motor.setVoltage(voltage);

        SmartDashboard.putBoolean("pitch/AtGoal", atGoal());
    }

    public void setGoal(Rotation2d position, Measure<Velocity<Angle>> velocity) {
        controller.setGoal(new TrapezoidProfile.State(position.getRadians(), velocity.in(RadiansPerSecond)));

        boolean isTop = MathUtil.isNear(position.getRadians(), Math.PI / 2, 0.05);
        deadband = isTop ? VERTICAL_PITCH_DEADBAND : DEFAULT_PITCH_DEADBAND;
    }

    public void setGoal(Rotation2d position) {
        setGoal(position, RadiansPerSecond.of(0));
    }

    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        controller.setConstraints(constraints);
    }

    public boolean atGoal() {
        return controller.atGoal() && Math.abs(controller.getGoal().velocity - getCurrentVelocity().in(RadiansPerSecond)) < 0.02;
    }

    public Rotation2d getGoal() {
        return Rotation2d.fromRadians(controller.getGoal().position);
    }

    public Rotation2d getCurrentPosition() {
        double angle = (Rotation2d.fromRotations(encoderPositionSignal.refresh().getValue()).getDegrees() - PIVOT_ENCODER_OFFSET);

        if (angle < -30) {
            angle += 360;
        } //todo: check if needed.

        return Rotation2d.fromDegrees(angle);
    }

    public Measure<Velocity<Angle>> getCurrentVelocity() {
        return RotationsPerSecond.of(encoderVelocitySignal.refresh().getValue());
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return controller.getConstraints();
    }

    public void stopMotor() {
        setGoal(Rotation2d.fromDegrees(0));
        motor.stopMotor();
    }

    private void configureSoftLimits() {
        motor.getEncoder().setPosition(getCurrentPosition().getRotations()); //initialize to current position

        Rotation2d FORWARD_PITCH_SOFT_LIMIT = Rotation2d.fromDegrees(90);
        Rotation2d REVERSE_PITCH_SOFT_LIMIT = Rotation2d.fromDegrees(-20);

        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float)FORWARD_PITCH_SOFT_LIMIT.getRotations());
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);

        motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float)REVERSE_PITCH_SOFT_LIMIT.getRotations());
        motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
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
}

