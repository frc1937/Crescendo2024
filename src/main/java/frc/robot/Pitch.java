package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.ShootingConstants.DEFAULT_PITCH_DEADBAND;
import static frc.robot.Constants.ShootingConstants.PITCH_KA;
import static frc.robot.Constants.ShootingConstants.PITCH_KD;
import static frc.robot.Constants.ShootingConstants.PITCH_KG;
import static frc.robot.Constants.ShootingConstants.PITCH_KP;
import static frc.robot.Constants.ShootingConstants.PITCH_KS;
import static frc.robot.Constants.ShootingConstants.PITCH_KV;
import static frc.robot.Constants.ShootingConstants.PITCH_MAX_ACCELERATION;
import static frc.robot.Constants.ShootingConstants.PITCH_MAX_VELOCITY;
import static frc.robot.Constants.ShootingConstants.PITCH_TRANSMISSION_RATIO;
import static frc.robot.Constants.ShootingConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DEGREES;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DIRECTION;
import static frc.robot.Constants.ShootingConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.ShootingConstants.PIVOT_ID;
import static frc.robot.Constants.ShootingConstants.PIVOT_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.VERTICAL_PITCH_DEADBAND;

public class Pitch {
    private final CANSparkFlex motor = new CANSparkFlex(PIVOT_ID, MotorType.kBrushless);
    private final RelativeEncoder relativeEncoder = motor.getEncoder();
    private final CANCoder absoluteEncoder = new CANCoder(PIVOT_CAN_CODER);
    private final ArmFeedforward feedforward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KA);
    private final ProfiledPIDController controller;

    private double deadband = DEFAULT_PITCH_DEADBAND;

    public Pitch() {
        // Configure the motor
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        // Configure the absolute encoder. Use the blocking methods because we configure
        // the relative encoder according to this one right away. 
        absoluteEncoder.configFactoryDefault(80);
        absoluteEncoder.configSensorDirection(true, 80);

        // Configure the relative encoder
        relativeEncoder.setPosition(getCurrentPosition().getRotations() * PITCH_TRANSMISSION_RATIO);

        // Configure a soft limit
        motor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, (float)Units.degreesToRotations(PIVOT_CONSTRAINT_DEGREES) * PITCH_TRANSMISSION_RATIO);
        motor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);

        controller = new ProfiledPIDController(
                PITCH_KP, 0, PITCH_KD,
                new TrapezoidProfile.Constraints(PITCH_MAX_VELOCITY, PITCH_MAX_ACCELERATION));
                // new TrapezoidProfile.Constraints(PITCH_MAX_VELOCITY, worstCaseAcceleration));

        controller.setTolerance(PIVOT_TOLERANCE);
    }

    public void periodic() {
        SmartDashboard.putNumber("pitch/CurrentAngle", getCurrentPosition().getDegrees());
        SmartDashboard.putNumber("pitch/Setpoint", Units.radiansToDegrees(controller.getSetpoint().position));
        SmartDashboard.putNumber("pitch/VelocitySetpoint", Units.radiansToDegrees(controller.getSetpoint().velocity));
        SmartDashboard.putNumber("pitch/relativePosition", getPositionFromRelativeEncoder().getDegrees());

        double velocitySetpoint = MathUtil.applyDeadband(
                controller.calculate(getCurrentPosition().getRadians()), deadband);

        double voltage = feedforward.calculate(getCurrentPosition().getRadians(), velocitySetpoint);
        motor.setVoltage(voltage);

        SmartDashboard.putBoolean("pitch/AtGoal", atGoal());
    }

    public void setGoal(Measure<Angle> position, Measure<Velocity<Angle>> velocity) {
        controller.setGoal(new TrapezoidProfile.State(position.in(Radians), velocity.in(RadiansPerSecond)));

        deadband = position.isNear(Degrees.of(90), 0.05) ? VERTICAL_PITCH_DEADBAND : DEFAULT_PITCH_DEADBAND;
    }

    public void setGoal(Rotation2d position) {
        setGoal(Radians.of(position.getRadians()), RadiansPerSecond.of(0));
    }

    public boolean atGoal() {
        // WARNING: this does not check whether the velocity goal was reached
        return controller.atGoal();
    }

    public Rotation2d getCurrentPosition() {
        double angle = (absoluteEncoder.getAbsolutePosition() - PIVOT_ENCODER_OFFSET);

        if (angle < -30) {
            angle += 360;
        }

        return Rotation2d.fromDegrees(angle);
    }

    private Rotation2d getPositionFromRelativeEncoder() {
        return Rotation2d.fromRotations(relativeEncoder.getPosition() / PITCH_TRANSMISSION_RATIO);
    }

    public Measure<Velocity<Angle>> getCurrentVelocity() {
        return DegreesPerSecond.of(absoluteEncoder.getVelocity());
    }

    public void stopMotor() {
        setGoal(Rotation2d.fromDegrees(0));
        motor.stopMotor();
    }
}
