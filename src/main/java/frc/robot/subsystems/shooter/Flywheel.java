// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.CANSparkMaxUtil;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class Flywheel {
    private CANSparkFlex motor;
    private RelativeEncoder encoder;

    private PIDController feedback;
    private SimpleMotorFeedforward feedforward;

    private Measure<Velocity<Angle>> goal;

    public Flywheel(int motorId, boolean invert, double kP, double kS, double kV, double kA) {
        seedFeeders(kP, kS, kV, kA);
        configureMotor(motorId, invert);
        configureEncoder();
    }

    private void seedFeeders(double kP, double kS, double kV, double kA) {
        feedback = new PIDController(kP, 0, 0);
        feedback.setTolerance(FLYWHEEL_TOLERANCE.in(RPM));

        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public void periodic() {
        logFlywheel();
        driveFlywheel();
    }

    public void setGoal(Measure<Velocity<Angle>> speed) {
        goal = speed;
        feedback.reset();
    }

    public Measure<Velocity<Angle>> getGoal() {
        if (goal == null)
            return RPM.of(0);

        return goal;
    }

    public boolean isAtGoal() {
        if (goal == null)
            return false;

        return getVelocity().isNear(goal, FLYWHEEL_TOLERANCE.in(RPM));
    }

    public Measure<Velocity<Angle>> getVelocity() {
        return RPM.of(encoder.getVelocity());
    }

    public void stopMotor() {
        setGoal(RPM.of(0));
        motor.stopMotor();
    }

    private void driveFlywheel() {
        if(goal == null || goal.in(RPM) == 0) return;

        double currentVelocityRPM = getVelocity().in(RPM);
        double goalVelocityRPM = goal.in(RPM);

        double feedbackOutput = feedback.calculate(currentVelocityRPM, goalVelocityRPM);
        double feedforwardOutput = feedforward.calculate(currentVelocityRPM, goalVelocityRPM, TIME_DIFFERENCE);
        //todo: This might be incorrect

        motor.setVoltage(feedbackOutput + feedforwardOutput);
    }

    private void logFlywheel() {
        SmartDashboard.putNumber("flywheel/" + motor.getDeviceId() + "/current velocity [RPM]", getVelocity().in(RPM));
        SmartDashboard.putNumber("flywheel/" + motor.getDeviceId() + "/goal [RPM]", getGoal().in(RPM));
    }

    private void configureEncoder() {
        encoder = motor.getEncoder(SparkRelativeEncoder.Type.kNoSensor, 7168);
    }

    private void configureMotor(int motorId, boolean invert) {
        motor = new CANSparkFlex(motorId, MotorType.kBrushless);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(invert);

        motor.enableVoltageCompensation(12);
        motor.setSmartCurrentLimit(40);

        CANSparkMaxUtil.setCANSparkBusUsage(motor, CANSparkMaxUtil.Usage.kVelocityOnly);

        motor.burnFlash();
    }
}
