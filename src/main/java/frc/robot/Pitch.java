package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.ShootingConstants.PITCH_KA;
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
    private final CANCoder encoder = new CANCoder(PIVOT_CAN_CODER);
    private final ArmFeedforward feedforward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KA);
    private final ProfiledPIDController controller;

    // Neglect the effect of gravity
    private final LinearSystem<N2,N1,N1> plant = LinearSystemId.identifyPositionSystem(PITCH_KV, PITCH_KA);
    private final KalmanFilter<N2,N1,N1> observer = new KalmanFilter<>(
        Nat.N2(),
        Nat.N1(),
        plant,
        VecBuilder.fill(Units.degreesToRadians(1.2), DegreesPerSecond.of(20).in(RadiansPerSecond)),
        VecBuilder.fill(Units.degreesToRadians(0.8)),
        0.02
    );
    private double voltage = 0;

    public Pitch() {
//        SmartDashboard.putNumber("pitch/p-value", 0);
//        SmartDashboard.putNumber("pitch/d-value", 0);

        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);
        motor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, PIVOT_CONSTRAINT_DEGREES);

        encoder.configFactoryDefault();
        encoder.configSensorDirection(true);
        
        var worstCaseAcceleration = RadiansPerSecond.per(Second).of(feedforward.maxAchievableAcceleration(7, 0, Double.MIN_NORMAL));

        controller = new ProfiledPIDController(
            PITCH_KP, 0, PITCH_KD,
            new TrapezoidProfile.Constraints(PITCH_MAX_VELOCITY, worstCaseAcceleration.in(RotationsPerSecond.per(Second)))
        );

        controller.setTolerance(PIVOT_TOLERANCE);
    }

    public void periodic() {
//        controller.setP(SmartDashboard.getNumber("pitch/p-value", 0));
//        controller.setD(SmartDashboard.getNumber("pitch/d-value", 0));

        observer.correct(VecBuilder.fill(voltage), VecBuilder.fill(Units.degreesToRadians(getRawPosition())));

        double velocitySetpoint = controller.calculate(observer.getXhat(0));

        voltage = feedforward.calculate(observer.getXhat(0), velocitySetpoint);
        motor.setVoltage(voltage);
        observer.predict(VecBuilder.fill(voltage), 0.02);
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
        return Rotation2d.fromRadians(observer.getXhat(0));
    }

    public double getRawPosition() {
        double angle = (encoder.getAbsolutePosition() - PIVOT_ENCODER_OFFSET);

        if (angle < -30) {
            angle += 360;
        }
        
        return angle;
    }

    public Measure<Velocity<Angle>> getCurrentVelocity() {
        return DegreesPerSecond.of(encoder.getVelocity());
    }

    public void stopMotor() {
        setPosition(Rotation2d.fromDegrees(0));
        motor.stopMotor();
    }
}