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
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Flywheel {
    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;
    private final PIDController feedback;
    private final SimpleMotorFeedforward feedforward;
    private final double p;

    private double feedforwardCorrection = 0;
    private Measure<Velocity<Angle>> setpoint;
    private Measure<Velocity<Angle>> velocity;

    public final Measure<Velocity<Angle>> theoreticalMaximumVelocity;

    public Flywheel(int motorId, boolean invert, double p, double s, double v, double a) {
        this.p = p;

        feedback = new PIDController(p, 0, 0);
        feedforward = new SimpleMotorFeedforward(s, v, a);

        feedback.setTolerance(ShooterConstants.FlywheelControlConstants.TOLERANCE);

        motor = configureMotor(motorId, invert);
        encoder = motor.getEncoder(SparkRelativeEncoder.Type.kNoSensor, 7168);

        theoreticalMaximumVelocity = RotationsPerSecond.of(feedforward.maxAchievableVelocity(12, 0));
    }

    public void periodic() {
        logFlywheel();
        driveFlywheel();
    }

    public void setSpeed(Measure<Velocity<Angle>> speed, double pScalar) {
        setpoint = speed;

        feedback.setSetpoint(speed.in(RotationsPerSecond));
        feedback.setP(p * pScalar);
        feedforwardCorrection = feedforward.calculate(speed.in(RotationsPerSecond));
    }

    public void setSpeed(Measure<Velocity<Angle>> speed) {
        setSpeed(speed, 1);
    }

    public Measure<Velocity<Angle>> getSetpoint() {
        if (setpoint != null)
            return setpoint;

        return RPM.of(0);
    }

    public boolean atSetpoint() {
        return feedback.atSetpoint();
    }

    public Measure<Velocity<Angle>> getSpeed() {
        return RPM.of(encoder.getVelocity());
    }

    public void stopMotor() {
        setSpeed(RPM.of(0));
        motor.stopMotor();
    }

    private CANSparkFlex configureMotor(int motorId, boolean invert) {
        CANSparkFlex sparkFlexMotor = new CANSparkFlex(motorId, MotorType.kBrushless);

        sparkFlexMotor.restoreFactoryDefaults();
        sparkFlexMotor.setIdleMode(IdleMode.kCoast);
        sparkFlexMotor.setInverted(invert);

        CANSparkMaxUtil.setCANSparkBusUsage(sparkFlexMotor, CANSparkMaxUtil.Usage.kVelocityOnly);

        return sparkFlexMotor;
    }

    private void driveFlywheel() {
        velocity = RPM.of(encoder.getVelocity());
        double feedbackCorrection = feedback.calculate(velocity.in(RotationsPerSecond));

        motor.setVoltage(feedbackCorrection + feedforwardCorrection);
    }

    private void logFlywheel() {
        if(velocity != null)
            SmartDashboard.putNumber("flywheel/" + motor.getDeviceId() + "/velocity [rpm]", velocity.in(RPM));

        SmartDashboard.putNumber("flywheel/" + motor.getDeviceId() + "/setpoint [rpm]", feedback.getSetpoint() * 60);
    }
}
