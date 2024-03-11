// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.simulation.EncoderDataJNI;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShootingConstants.FlywheelControlConstants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.ShootingConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DEGREES;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DIRECTION;

public class Pitch {
    private final PIDController feedback;
    private final CANSparkFlex motor;
    private final CANCoder encoder;
    private final ArmFeedforward feedforward;
    private final TrapezoidProfile motionProfile = new TrapezoidProfile(PITCH_CONSTRAINTS);
    private final TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);

    private double feedforwardCorrection = 0;

    public Pitch(int motorId, double p, double d, double s, double g, double v) {
        feedback = new PIDController(p, 0, d);
        feedforward = new ArmFeedforward(s, g, v);

        motor = new CANSparkFlex(motorId, MotorType.kBrushless);
        motor.restoreFactoryDefaults();

        motor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);
        motor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, PIVOT_CONSTRAINT_DEGREES);
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        // WARNING FIXME XXX TODO Shout at Uriel
        // feedback.setTolerance(FlywheelControlConstants.TOLERANCE);

        encoder = new CANCoder(PIVOT_CAN_CODER);
        encoder.configFactoryDefault();
        encoder.configSensorDirection(true);
    }

    public void periodic() {
        // Calculate the position and velocity setpoints
        TrapezoidProfile.State setpoint = motionProfile.calculate(0.02, getCurrentState(), goal);

        // Derive the acceleration setpoint from the current and future velocity
        double accelerationRadPerSecSquared = (setpoint.velocity - getCurrentVelocity().in(RadiansPerSecond)) / 0.02;

        // Control the motor
        double velocityFeedforward = feedforward.calculate(setpoint.position, setpoint.velocity, accelerationRadPerSecSquared);
        double positionFeedback = feedback.calculate(getPitch().getRotations(), Units.radiansToRotations(setpoint.position));
        motor.setVoltage(velocityFeedforward + positionFeedback);
    }

    public void setGoal(Measure<Angle> position, Measure<Velocity<Angle>> velocity) {
        goal.position = position.in(Radians);
        goal.velocity = velocity.in(RadiansPerSecond);
    }

    @Deprecated
    public void setPosition(Rotation2d rotation2d) {
        setGoal(Radians.of(rotation2d.getRadians()), RadiansPerSecond.of(0));
    }

    public boolean atSetpoint() {
        return false;
        // WARNING: Unimplemented
        // return feedback.atSetpoint();
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition() - 343);  // TODO move 343 to Constants.java
    }

    public Measure<Velocity<Angle>> getCurrentVelocity() {
        return DegreesPerSecond.of(encoder.getVelocity());
    }

    public void stopMotor() {
        setPosition(Rotation2d.fromDegrees(0));
        motor.stopMotor();
    }

    public TrapezoidProfile.State getCurrentState() {
        return new TrapezoidProfile.State(getPitch().getRadians(), getCurrentVelocity().in(RadiansPerSecond));
    }
}
