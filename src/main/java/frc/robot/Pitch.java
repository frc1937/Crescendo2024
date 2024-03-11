// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
import static frc.robot.Constants.ShootingConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DEGREES;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DIRECTION;
import static frc.robot.Constants.ShootingConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.ShootingConstants.PIVOT_TOLERANCE;

public class Pitch {
    private final PIDController feedback;
    private final CANSparkFlex motor;
    private final CANCoder encoder;
    private final ArmFeedforward feedforward;
    private final double goal = 0.7;
//    private final TrapezoidProfile.State goal = new TrapezoidProfile.State(0.7, 0);
//    private boolean goalIsSetpoint = false;

    public Pitch(int motorId, double p, double d, double s, double g, double v) {
        feedback = new PIDController(p/5, 0, d/5);
//        velocityController = new PIDController(2, 0, 0);
        feedforward = new ArmFeedforward(s, g, v);

        motor = new CANSparkFlex(motorId, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);
        motor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, PIVOT_CONSTRAINT_DEGREES);

        feedback.setTolerance(PIVOT_TOLERANCE);

        encoder = new CANCoder(PIVOT_CAN_CODER);
        encoder.configFactoryDefault();
        encoder.configSensorDirection(true);

//        SmartDashboard.putNumber("pitch/P-value", 1);
//        SmartDashboard.putNumber("pitch/D-value", 0);
//        SmartDashboard.putNumber("pitch/goal-value", 0);
    }

    public void periodic() {
//        velocityController = new PIDController(SmartDashboard.getNumber("pitch/P-value", 0)
//                , 0, SmartDashboard.getNumber("pitch/D-value", 0));
//
//        SmartDashboard.putNumber("GOAL Position [deg]", goal.position);
//        SmartDashboard.putNumber("GOAL Current Deg [deg]", getPitch().getDegrees());

        // Calculate the position and velocity setpoints

        //TrapezoidProfile.State setpoint = motionProfile.calculate(0.02, getCurrentState(), goal);
        //goalIsSetpoint = goal.equals(setpoint);
//        goalIsSetpoint = false;

//        double velocitySetpoint = 0;
//
//        if (!feedback.atSetpoint()) {
//            velocitySetpoint = velocityController.calculate(getCurrentState().position, goal.position);
//            velocitySetpoint = MathUtil.clamp(velocitySetpoint, -0.8, 0.8);
//        }
//        double velocitySetpoint = MathUtil.clamp((goal.position - getCurrentState().position) *
//                SmartDashboard.getNumber("pitch/P-value", 2), -0.2, 0.2);

//        SmartDashboard.putNumber("Setpoint Position [deg]", setpoint.position);
//        SmartDashboard.putNumber("Setpoint Velocity [RadPS]", velocitySetpoint);

        // Derive the acceleration setpoint from the current and future velocity
//        double accelerationRadPerSecSquared = (velocitySetpoint - getCurrentVelocity().in(RadiansPerSecond)) / 0.02;

        // Control the motor
        double voltage;

        double error = goal - getCurrentState().position;
        if (error < -0.1) {
            voltage = feedforward.calculate(getCurrentState().position, -0.6, 0);
        } else if (error > 0.1) {
            voltage = feedforward.calculate(getCurrentState().position, 0.6, 0);
        } else {
            voltage = feedback.calculate(getPitch().getRotations(), Units.radiansToRotations(goal));
        }
//        if (Math.abs(getCurrentState().position - goal) > 0.1)
//        double velocityFeedforward = feedforward.calculate(getCurrentState().position, velocitySetpoint, 0);

//        if (goalIsSetpoint)
//        double positionFeedback = feedback.calculate(getPitch().getRotations(), Units.radiansToRotations(setpoint.position));

//        SmartDashboard.putNumber("FeedForwardVel [velocity units LOL]", velocityFeedforward);
//        SmartDashboard.putNumber("FeedBack [position in rotations]", positionFeedback);
//        SmartDashboard.putBoolean("FeedLoopHas Reached Goal!! [BOOL]", atGoal());

        motor.setVoltage(voltage);
    }

    public void setGoal(Measure<Angle> position, Measure<Velocity<Angle>> velocity) {
//        goal.position = position.in(Radians);
//        goal.velocity = velocity.in(RadiansPerSecond);

        SmartDashboard.putNumber("Position Goal [deg]", position.in(Degrees));
        SmartDashboard.putNumber("Position Velocity [RadPS]", velocity.in(RadiansPerSecond));
        //todo: check setpoint, velocity and gaol
    }

    @Deprecated
    public void setPosition(Rotation2d rotation2d) {
        setGoal(Radians.of(
                SmartDashboard.getNumber("pitch/goal-value", 0)
//                rotation2d.getRadians()
        ), RadiansPerSecond.of(0));
    }

    public boolean atGoal() {
        // WARNING: this does not check whether the velocity goal was reached
        return /*goalIsSetpoint && */feedback.atSetpoint();
    }

    @Deprecated
    public boolean atSetpoint() {
        return atGoal();
    }

    public Rotation2d getPitch() {
        double angle = (encoder.getAbsolutePosition() - PIVOT_ENCODER_OFFSET);

        if (angle < -30) {
            angle += 360;
        }

        return Rotation2d.fromDegrees(angle);
    }

    public Measure<Velocity<Angle>> getCurrentVelocity() {
        return DegreesPerSecond.of(encoder.getVelocity());
    }

    public void stopMotor() {
        setPosition(Rotation2d.fromDegrees(0));
        motor.stopMotor();
    }

    private TrapezoidProfile.State getCurrentState() {
        return new TrapezoidProfile.State(getPitch().getRadians(), getCurrentVelocity().in(RadiansPerSecond));
    }
}
