// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.Constants.ShootingConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DEGREES;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DIRECTION;
import static frc.robot.Constants.ShootingConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.ShootingConstants.PIVOT_G;
import static frc.robot.Constants.ShootingConstants.PIVOT_P;
import static frc.robot.Constants.ShootingConstants.PIVOT_S;
import static frc.robot.Constants.ShootingConstants.PIVOT_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.PIVOT_V;

public class Pivot {
    private final CANSparkFlex motor;
    private final CANCoder encoder;
    private final PIDController feedback = new PIDController(PIVOT_P * 60, 0, 0);
    private final ArmFeedforward feedforward = new ArmFeedforward(PIVOT_S, PIVOT_G, PIVOT_V);
    private double feedforwardCorrection = 0;

    public Pivot(int motorId, boolean invert) {
        motor = new CANSparkFlex(motorId, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(invert);

        motor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);
        motor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, PIVOT_CONSTRAINT_DEGREES);

        feedback.setTolerance(PIVOT_TOLERANCE);

        encoder = new CANCoder(PIVOT_CAN_CODER);
        encoder.configFactoryDefault();
    }

    public void periodic() {
        double currentAngle = PIVOT_ENCODER_OFFSET - encoder.getAbsolutePosition();
        double feedbackCorrection = feedback.calculate(currentAngle);

        motor.setVoltage(feedbackCorrection + feedforwardCorrection);
    }

    public void setPivotAngle(Rotation2d rotation2d) {
        feedback.setSetpoint(rotation2d.getDegrees());
        feedforwardCorrection = feedforward.calculate(rotation2d.getDegrees(), 0);
    }

    public boolean atSetpoint() {
        return feedback.atSetpoint();
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition() - PIVOT_ENCODER_OFFSET);
    }

    public void stopMotor() {
        motor.stopMotor();
    }
}
