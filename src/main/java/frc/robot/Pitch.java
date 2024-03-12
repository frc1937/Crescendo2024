// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.ShootingConstants.PITCH_KD;
import static frc.robot.Constants.ShootingConstants.PITCH_KG;
import static frc.robot.Constants.ShootingConstants.PITCH_KP;
import static frc.robot.Constants.ShootingConstants.PITCH_KS;
import static frc.robot.Constants.ShootingConstants.PITCH_KV;
import static frc.robot.Constants.ShootingConstants.PITCH_MAX_VELOCITY;
import static frc.robot.Constants.ShootingConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DEGREES;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DIRECTION;
import static frc.robot.Constants.ShootingConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.ShootingConstants.PIVOT_ID;
import static frc.robot.Constants.ShootingConstants.PIVOT_TOLERANCE;

public class Pitch {
    private final CANSparkFlex motor = new CANSparkFlex(PIVOT_ID, MotorType.kBrushless);
    private final CANCoder encoder;

    private final ArmFeedforward feedforward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KS);
    private final ProfiledPIDController controller;

    // private final TrapezoidProfile.State goal = new TrapezoidProfile.State();

    public Pitch() {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);
        motor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, PIVOT_CONSTRAINT_DEGREES);

        encoder = new CANCoder(PIVOT_CAN_CODER);
        encoder.configFactoryDefault();
        encoder.configSensorDirection(true);
        
        var wrostCaseAcceleration = RadiansPerSecond.per(Second).of(feedforward.maxAchievableAcceleration(7, 0, Double.MIN_NORMAL));
        controller = new ProfiledPIDController(
            PITCH_KP, 0, PITCH_KD,
            new TrapezoidProfile.Constraints(PITCH_MAX_VELOCITY, wrostCaseAcceleration.in(RotationsPerSecond.per(Second)))
        );
        controller.setTolerance(PIVOT_TOLERANCE);
    }

    public void periodic() {
        double velocitySetpoint = controller.calculate(getCurrentPosition().getRadians());
        // setpoint.velocity = MathUtil.clamp(setpoint.velocity, -0.1, 0.1);
        double voltage = feedforward.calculate(getCurrentPosition().getRadians(), velocitySetpoint);
        motor.setVoltage(voltage);
    }

    public void setGoal(Measure<Angle> position, Measure<Velocity<Angle>> velocity) {
        controller.setGoal(new TrapezoidProfile.State(position.in(Radians), velocity.in(RadiansPerSecond)));
    }

    @Deprecated
    public void setPosition(Rotation2d rotation2d) {
        setGoal(Radians.of(rotation2d.getRadians()), RadiansPerSecond.of(0));
    }

    public boolean atGoal() {
        // WARNING: this does not check whether the velocity goal was reached
        return controller.atGoal();
    }

    @Deprecated
    public boolean atSetpoint() {
        return atGoal();
    }

    public Rotation2d getCurrentPosition() {
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
}
