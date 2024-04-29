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

    public final Measure<Velocity<Angle>> theoreticalMaximumVelocity;

    public Flywheel(int motorId, boolean invert, double p, double s, double v, double a) {
        this.p = p;

        feedback = new PIDController(p, 0, 0);
        feedforward = new SimpleMotorFeedforward(s, v, a);

        motor = new CANSparkFlex(motorId, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(invert);

        feedback.setTolerance(ShooterConstants.FlywheelControlConstants.TOLERANCE);

        encoder = motor.getEncoder(SparkRelativeEncoder.Type.kNoSensor, 7168);

        theoreticalMaximumVelocity = RotationsPerSecond.of(feedforward.maxAchievableVelocity(12, 0));
    }

    public void periodic() {
        Measure<Velocity<Angle>> velocity = RPM.of(encoder.getVelocity());
        double feedbackCorrection = feedback.calculate(velocity.in(RotationsPerSecond));

        SmartDashboard.putNumber("flywheel/" + motor.getDeviceId() + "/velocity [rpm]", velocity.in(RPM));
        SmartDashboard.putNumber("flywheel/" + motor.getDeviceId() + "/setpoint [rpm]", feedback.getSetpoint() * 60);

        motor.setVoltage(feedbackCorrection + feedforwardCorrection);
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
        if(setpoint != null)
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
}
