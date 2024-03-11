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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShootingConstants.FlywheelControlConstants;

import static frc.robot.Constants.ShootingConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DEGREES;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DIRECTION;

public class Pitch {
    private final PIDController feedback;
    private final CANSparkFlex motor;
    private final CANCoder encoder;
    private final ArmFeedforward feedforward;
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
        Rotation2d rotation2d = Rotation2d.fromDegrees(encoder.getAbsolutePosition());
        double feedbackCorrection = feedback.calculate(rotation2d.getRotations());

        SmartDashboard.putNumber("Voltage Pitch", -(feedbackCorrection + feedforwardCorrection));

        motor.setVoltage(feedbackCorrection + feedforwardCorrection);
    }

    public void setPosition(Rotation2d rotation2d) {
        feedback.setSetpoint(rotation2d.getRadians());
        feedforwardCorrection = feedforward.calculate(rotation2d.getRadians(), encoder.getVelocity());
    }

    public boolean atSetpoint() {
        return feedback.atSetpoint();
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition() - 343);  // TODO move 343 to Constants.java
    }

    public Rotation2d getSetpoint() {
        return Rotation2d.fromRadians(feedback.getSetpoint());
    }

    public void stopMotor() {
        setPosition(Rotation2d.fromDegrees(0));
        motor.stopMotor();
    }
}
