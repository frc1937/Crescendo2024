// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CANSparkMaxUtil;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.VOLTAGE_COMPENSATION_SATURATION;
import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_TOLERANCE;

public class Flywheel {
    private CANSparkFlex motor;
    private RelativeEncoder encoder;

    private PIDController feedback;
    private SimpleMotorFeedforward feedforward;

    private Measure<Velocity<Angle>> goal;

    private double feedforwardOutput, feedbackOutput;

    public Flywheel(int motorId, boolean invert, double kP, double kS, double kV) {
        seedFeeders(kP, kS, kV);
        configureMotor(motorId, invert);
        configureEncoder();
    }

    public void periodic() {
        logFlywheel();
        driveFlywheel();
    }

    public void setGoal(Measure<Velocity<Angle>> speed) {
        goal = speed;

        feedback.reset();
        feedback.setSetpoint(speed.in(RotationsPerSecond));

        feedforwardOutput = feedforward.calculate(goal.in(RotationsPerSecond));
    }

    public boolean isAtGoal() {
        if (goal == null)
            return false;

        //todo: wtf is going on here.
        return Math.abs(getVelocity().in(RotationsPerSecond) - goal.in(RotationsPerSecond)) < 50;
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return RPM.of(encoder.getVelocity());
    }

    public void stopMotor() {
        motor.stopMotor();
    }

    /**
     * Returns the voltage the motor
     *
     * @return the amount of voltage the motor uses
     * @Units Volts
     */
    public Measure<Voltage> getVoltage() {
        return Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    }

    public void setRawVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public Measure<Angle> getPosition() {
        return Rotations.of(encoder.getPosition());
    }

    private void driveFlywheel() {
        if (goal == null || goal.in(RotationsPerSecond) == 0) return;

        double currentVelocityRPS = getVelocity().in(RotationsPerSecond);

        feedbackOutput = feedback.calculate(currentVelocityRPS);

        motor.setVoltage(feedbackOutput + feedforwardOutput);
    }

    private void seedFeeders(double kP, double kS, double kV) {
        feedback = new PIDController(kP, 0, 0);
        feedback.setTolerance(FLYWHEEL_TOLERANCE.in(RotationsPerSecond));

        feedforward = new SimpleMotorFeedforward(kS, kV, 0);
    }

    private void logFlywheel() {
        if(goal != null)
            SmartDashboard.putNumber("flywheel/" + motor.getDeviceId() + "/goalVelocity [RPM]", goal.in(RPM));

        SmartDashboard.putNumber("flywheel/" + motor.getDeviceId() + "/currentVelocity [RPM]", getVelocity().in(RPM));
        SmartDashboard.putNumber("flywheel/" + motor.getDeviceId() + "/feedbackOutput [Volts]", feedbackOutput);
        SmartDashboard.putNumber("flywheel/" + motor.getDeviceId() + "/feedforwardOutput [Volts]", feedforwardOutput);
        SmartDashboard.putNumber("flywheel/" + motor.getDeviceId() + "/voltage [Volts]", getVoltage().in(Volt));
    }

    private void configureEncoder() {
        encoder = motor.getEncoder();
    }

    private void configureMotor(int motorId, boolean invert) {
        motor = new CANSparkFlex(motorId, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(invert);

        motor.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        motor.setSmartCurrentLimit(30);

        CANSparkMaxUtil.setCANSparkBusUsage(motor, CANSparkMaxUtil.Usage.kVelocityOnly);

        motor.burnFlash();
    }
}
