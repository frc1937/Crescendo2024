package frc.robot;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
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

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.constants.Constants.ShootingConstants.*;

public class Pitch {
    private final CANSparkFlex motor = new CANSparkFlex(PIVOT_ID, MotorType.kBrushless);
    private final RelativeEncoder relativeEncoder = motor.getEncoder();
    private final CANcoder absoluteEncoder = new CANcoder(PIVOT_CAN_CODER);
    private final ArmFeedforward feedforward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KA);
    private final ProfiledPIDController controller;

    private double deadband = DEFAULT_PITCH_DEADBAND;

    public Pitch() {
        // Configure the motor
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        // Configure the absolute encoder. Use the blocking methods because we configure
        // the relative encoder according to this one right away.
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        //todo: Check if this is the equivalent of true

        absoluteEncoder.getConfigurator().apply(config, 0.080);

        // Configure the relative encoder
        relativeEncoder.setPosition(getCurrentPosition().getRotations() * PITCH_TRANSMISSION_RATIO);

        // Configure a soft limit
//        motor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, (float)Units.degreesToRotations(PIVOT_CONSTRAINT_DEGREES) * PITCH_TRANSMISSION_RATIO);
//        motor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);

        controller = new ProfiledPIDController(
                PITCH_KP, 0, PITCH_KD,
                new TrapezoidProfile.Constraints(PITCH_MAX_VELOCITY, PITCH_MAX_ACCELERATION));

        controller.setTolerance(PIVOT_TOLERANCE);
    }

    public void periodic() {
        SmartDashboard.putNumber("pitch/CurrentAngle", getCurrentPosition().getDegrees());
        SmartDashboard.putNumber("pitch/Goal", Units.radiansToDegrees(controller.getGoal().position));
        SmartDashboard.putNumber("pitch/VelocitySetpoint", Units.radiansToDegrees(controller.getSetpoint().velocity));
        SmartDashboard.putNumber("pitch/relativePosition", getPositionFromRelativeEncoder().getDegrees());

        double velocitySetpoint = MathUtil.applyDeadband(
                controller.calculate(getCurrentPosition().getRadians()), deadband);

        double voltage = feedforward.calculate(getCurrentPosition().getRadians(), velocitySetpoint);
        motor.setVoltage(voltage);

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

    public Rotation2d getCurrentPosition() {
        double angle = (absoluteEncoder.getAbsolutePosition().getValue() - PIVOT_ENCODER_OFFSET);

        if (angle < -30) {
            angle += 360;
        }

        return Rotation2d.fromDegrees(angle);
    }

    private Rotation2d getPositionFromRelativeEncoder() {
        return Rotation2d.fromRotations(relativeEncoder.getPosition() / PITCH_TRANSMISSION_RATIO);
    }

    public Measure<Velocity<Angle>> getCurrentVelocity() {
        return DegreesPerSecond.of(absoluteEncoder.getVelocity().getValue());
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return controller.getConstraints();
    }

    public void stopMotor() {
        setGoal(Rotation2d.fromDegrees(0));
        motor.stopMotor();
    }
}

