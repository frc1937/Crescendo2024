// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.generic.Feedforward;
import frc.lib.generic.Properties;
import frc.lib.generic.motor.GenericSpark;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_TOLERANCE;

public class Flywheel {
    private Motor motor;

    private PIDController feedback;
    private Feedforward feedforward;

    private Measure<Velocity<Angle>> goal;

    private double feedforwardOutput, feedbackOutput;

    public Flywheel(int motorId, boolean invert, double kP, double kS, double kV) {
        seedFeeders(kP, kS, kV);
        configureMotor(motorId, invert);
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
        return RPM.of(motor.getMotorVelocity());
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
        return Volts.of(motor.getVoltage() * 12);
    }

    public void setRawVoltage(double voltage) {
        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, voltage);
    }

    public Measure<Angle> getPosition() {
        return Rotations.of(motor.getMotorPosition());
    }

    private void driveFlywheel() {
        if (goal == null || goal.in(RotationsPerSecond) == 0) return;

        double currentVelocityRPS = getVelocity().in(RotationsPerSecond);

        feedbackOutput = feedback.calculate(currentVelocityRPS);

        motor.setOutput(MotorProperties.ControlMode.VOLTAGE, feedbackOutput + feedforwardOutput);
    }

    private void seedFeeders(double kP, double kS, double kV) {
        feedback = new PIDController(kP, 0, 0);
        feedback.setTolerance(FLYWHEEL_TOLERANCE.in(RotationsPerSecond));

        feedforward = new Feedforward(Properties.FeedforwardType.SIMPLE, kS, kV, 0);
    }

    private void logFlywheel() {
        if (goal != null)
            SmartDashboard.putNumber("flywheel/" + motor.getDeviceID() + "/goalVelocity [RPM]", goal.in(RPM));

        SmartDashboard.putNumber("flywheel/" + motor.getDeviceID() + "/currentVelocity [RPM]", getVelocity().in(RPM));
        SmartDashboard.putNumber("flywheel/" + motor.getDeviceID() + "/feedbackOutput [Volts]", feedbackOutput);
        SmartDashboard.putNumber("flywheel/" + motor.getDeviceID() + "/feedforwardOutput [Volts]", feedforwardOutput);
        SmartDashboard.putNumber("flywheel/" + motor.getDeviceID() + "/voltage [Volts]", getVoltage().in(Volt));
    }

    private void configureMotor(int motorId, boolean invert) {
        motor = new GenericSpark(motorId, MotorProperties.SparkType.FLEX);

        MotorConfiguration configuration = new MotorConfiguration();

        configuration.idleMode = MotorProperties.IdleMode.COAST;
        configuration.inverted = invert;

        configuration.supplyCurrentLimit = 80;
        configuration.statorCurrentLimit = 100;

        motor.configure(configuration);

        motor.setSignalUpdateFrequency(Properties.SignalType.VELOCITY, 50);
    }
}
