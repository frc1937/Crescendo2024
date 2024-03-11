package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShootingConstants.CONSIDERED_NOISELESS_THRESHOLD;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_FF;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_LEFT_ID;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_MAX_RPM;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_P;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_RIGHT_ID;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_VELOCITY_TOLERANCE;
import static frc.robot.Constants.ShootingConstants.KICKER_ID;

public class ShooterSubsystem extends SubsystemBase {
    private final DigitalInput beamBreaker = new DigitalInput(0);
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(KICKER_ID);
    private final CANSparkFlex flywheelMaster = new CANSparkFlex(FLYWHEEL_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex flywheelSlave = new CANSparkFlex(FLYWHEEL_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);

    //            pivotMotor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder flywheelEncoder = flywheelSlave.getEncoder(SparkRelativeEncoder.Type.kNoSensor, 7168);
//            pivotInternalEncoder = pivotMotor.getEncoder(SparkRelativeEncoder.Type.kNoSensor, 7168);
//    private final CANCoder pivotEncoder = new CANCoder(PIVOT_CAN_CODER);
    private final SparkPIDController flywheelsController;
    private double pivotSetpoint = 0, targetFlywheelVelocity = 0;
    private int consecutiveNoteInsideSamples = 0;

        // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

    public ShooterSubsystem() {
        configureSRXMotor(kickerMotor);

        flywheelsController = flywheelMaster.getPIDController();
        configureFlywheelMotor(flywheelMaster);

        configureFlywheelMotor(flywheelSlave);

//        flywheelSlave.follow(flywheelMaster, true);
//
//        pitchController = pivotMotor.getPIDController();
//        setupPivotController();
    }

    @Override
    public void periodic() {
//        double currentAngle = -(pivotEncoder.getPosition() - PIVOT_ENCODER_OFFSET);
//        pivotInternalEncoder.setPosition(currentAngle);

        /* FOR DEBUGGING, REMOVE */
        SmartDashboard.putNumber("Target Flywheel RPM", targetFlywheelVelocity);
//        SmartDashboard.putNumber("Current angle", currentAngle);
        SmartDashboard.putNumber("Pivot setpoint", pivotSetpoint);
        SmartDashboard.putBoolean("Does see note", doesSeeNote());
        SmartDashboard.putNumber("Flywheel RPM", flywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Flywheel ANGLE", flywheelEncoder.getPosition());

//        if (pivotSetpoint < 80) {
//            // Front region
//            if (pivotSetpoint >= currentAngle) {
//                // Up-moving PID
//                pitchController.setReference(pivotSetpoint, CANSparkBase.ControlType.kPosition, 0);
//            } else {
//                // Down-moving PID
//                pitchController.setReference(pivotSetpoint, CANSparkBase.ControlType.kPosition, 1);
//            }
//        } else if (pivotSetpoint > SHOOTER_VERTICAL_ANGLE * 2 - 80) {
//            // Rear region
//            if (pivotSetpoint <= currentAngle) {
//                // Up-moving PID
//                pitchController.setReference(pivotSetpoint, CANSparkBase.ControlType.kPosition, 0);
//            } else {
//                // Down-moving PID
//                pitchController.setReference(pivotSetpoint, CANSparkBase.ControlType.kPosition, 1);
//            }
//        } else {
//            // Top region
//            pitchController.setReference(pivotSetpoint, ControlType.kPosition, 2);
//        }

        if(doesSeeNote()) {
            consecutiveNoteInsideSamples++;
        } else {
            consecutiveNoteInsideSamples = 0;
        }
    }

    public boolean doesSeeNoteNoiseless() {
        return consecutiveNoteInsideSamples >= CONSIDERED_NOISELESS_THRESHOLD;
    }

    public void setKickerSpeed(double speed) {
        kickerMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopKicker() {
        kickerMotor.stopMotor();
    }

    public void setFlywheelSpeed(double targetFlywheelVelocity, boolean pid) {
        this.targetFlywheelVelocity = targetFlywheelVelocity;

        if (pid) {
            flywheelsController.setReference(targetFlywheelVelocity, ControlType.kVelocity);
        } else {
            flywheelMaster.set(targetFlywheelVelocity / FLYWHEEL_MAX_RPM);
        }
    }

    public void stopFlywheels() {
        targetFlywheelVelocity = 0;
        flywheelMaster.stopMotor();
    }

    public boolean areFlywheelsReady() {
        return Math.abs(Math.abs(flywheelEncoder.getVelocity()) - Math.abs(targetFlywheelVelocity)) <= FLYWHEEL_VELOCITY_TOLERANCE;
    }

    public boolean hasPivotArrived() {
        return false;
//        return Math.abs(pivotSetpoint - pivotInternalEncoder.getPosition()) < 1.5;
    }

    public void setPivotAngle(Rotation2d rotation2d) {
//        pivotSetpoint = rotation2d.getDegrees();
    }

    public void setFlywheelsVoltage(Measure<Voltage> volts) {
        flywheelSlave.setVoltage(volts.in(Volts));
    }

    public void logFlywheels(SysIdRoutineLog log) {
        log.motor("flywheel")
            .voltage(m_appliedVoltage.mut_replace(
                    flywheelSlave.getAppliedOutput() * flywheelSlave.getBusVoltage(), Volts))
            .angularPosition(m_angle.mut_replace(flywheelEncoder.getPosition(), Rotations))
            .angularVelocity(
                m_velocity.mut_replace(flywheelEncoder.getVelocity(), RPM));
    }

    private void configureCanCoder(CANCoder encoder) {
        encoder.configFactoryDefault();
    }

    private void configureFlywheelMotor(CANSparkFlex motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast);

        flywheelsController.setP(FLYWHEEL_P);
        flywheelsController.setD(0.000017);
        flywheelsController.setFF(FLYWHEEL_FF);
    }

    private void configureSRXMotor(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
    }

    private void configurePivotMotor(CANSparkFlex motor) {
        configureFlywheelMotor(motor);

//        pivotMotor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);
//        pivotMotor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, PIVOT_CONSTRAINT_DEGREES);
//        pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    private void setupPivotController() {
//        pitchController.setP(PIVOT_UP_P, 0);
//        pitchController.setFF(PIVOT_UP_FF, 0);
//
//        pitchController.setP(PIVOT_DOWN_P, 1);
//        pitchController.setFF(PIVOT_DOWN_FF, 1);
//
//        pitchController.setP(PIVOT_HIGH_P, 2);
//        pitchController.setD(PIVOT_HIGH_D, 2);
//        pitchController.setFF(PIVOT_HIGH_FF, 2);
//
//        pitchController.setOutputRange(PIVOT_RANGE_MIN, PIVOT_RANGE_MAX);
//
//        pivotInternalEncoder.setPosition(pivotEncoder.getAbsolutePosition());
//        pivotEncoder.setPosition(pivotEncoder.getAbsolutePosition());
    }

    private boolean doesSeeNote() {
        return !beamBreaker.get();
    }
}
