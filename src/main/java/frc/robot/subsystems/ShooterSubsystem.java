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
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Flywheel;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShootingConstants.CONSIDERED_NOISELESS_THRESHOLD;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_LEFT_ID;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_RIGHT_ID;
import static frc.robot.Constants.ShootingConstants.KICKER_ID;
import static frc.robot.Constants.ShootingConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DEGREES;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DIRECTION;
import static frc.robot.Constants.ShootingConstants.PIVOT_DOWN_FF;
import static frc.robot.Constants.ShootingConstants.PIVOT_DOWN_P;
import static frc.robot.Constants.ShootingConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.ShootingConstants.PIVOT_HIGH_D;
import static frc.robot.Constants.ShootingConstants.PIVOT_HIGH_FF;
import static frc.robot.Constants.ShootingConstants.PIVOT_HIGH_P;
import static frc.robot.Constants.ShootingConstants.PIVOT_ID;
import static frc.robot.Constants.ShootingConstants.PIVOT_RANGE_MAX;
import static frc.robot.Constants.ShootingConstants.PIVOT_RANGE_MIN;
import static frc.robot.Constants.ShootingConstants.PIVOT_UP_FF;
import static frc.robot.Constants.ShootingConstants.PIVOT_UP_P;
import static frc.robot.Constants.ShootingConstants.SHOOTER_VERTICAL_ANGLE;
import static edu.wpi.first.units.MutableMeasure.mutable;


public class ShooterSubsystem extends SubsystemBase {
    private final DigitalInput beamBreaker = new DigitalInput(0);
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(KICKER_ID);
    private final CANSparkFlex pivotMotor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder pivotInternalEncoder = pivotMotor.getEncoder(SparkRelativeEncoder.Type.kNoSensor, 7168);
    private final CANCoder pivotEncoder = new CANCoder(PIVOT_CAN_CODER);

    private final Flywheel rightFlywheel = new Flywheel(FLYWHEEL_RIGHT_ID, true);
    private final Flywheel leftFlywheel = new Flywheel(FLYWHEEL_LEFT_ID, false);

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));


    private double pivotSetpoint = 0, targetFlywheelVelocity = 0;
    private int consecutiveNoteInsideSamples = 0;

    public ShooterSubsystem() {
        configureSRXMotor(kickerMotor);
        configurePivotMotor(pivotMotor);
        configureCANCoder(pivotEncoder);
    }

    @Override
    public void periodic() {
        double currentAngle = -(pivotEncoder.getPosition() - PIVOT_ENCODER_OFFSET);
        pivotInternalEncoder.setPosition(currentAngle);

        /* FOR DEBUGGING, REMOVE */
        SmartDashboard.putNumber("Target Flywheel RPM", targetFlywheelVelocity);
        SmartDashboard.putNumber("Current angle", currentAngle);
        SmartDashboard.putNumber("Pivot setpoint", pivotSetpoint);
        SmartDashboard.putNumber("left flywheel rpm", leftFlywheel.getSpeed().in(RPM));
        SmartDashboard.putBoolean("Does see note", doesSeeNote());

        if(doesSeeNote()) {
            consecutiveNoteInsideSamples++;
        } else {
            consecutiveNoteInsideSamples = 0;
        }

        leftFlywheel.periodic();
        rightFlywheel.periodic();
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

    public void setFlywheelsSpeed(Measure<Velocity<Angle>> speed) {
        rightFlywheel.setSpeed(speed);
        leftFlywheel.setSpeed(speed);
    }

    /**
     * Rotate the flywheels to certain speeds s.t. NOTEs will be released with
     * certain speed and rotation
     * 
     * @param speed the average target speed of both flywheels
     * @param spin a value in range [0, 1] where (1 - spin) = (right speed / left speed). Thus,
     *             the difference between the left and right speeds is proprtional to
     *             {@code speed}.
     */
    public void setFlywheelsSpeed(Measure<Velocity<Angle>> speed, double spin) {
        var leftSpeed = speed.times(2).divide(2.d - spin);
        var rightSpeed = leftSpeed.times(1.d - spin);
        rightFlywheel.setSpeed(rightSpeed);
        leftFlywheel.setSpeed(leftSpeed);
    }

    public void stopFlywheels() {
        // targetFlywheelVelocity = 0;
        leftFlywheel.stopMotor();
        rightFlywheel.stopMotor();
    }

    public boolean areFlywheelsReady() {
        return leftFlywheel.atSetpoint() && rightFlywheel.atSetpoint();
        // return Math.abs(Math.abs(flywheelEncoder.getVelocity()) - Math.abs(targetFlywheelVelocity)) <= FLYWHEEL_VELOCITY_TOLERANCE;
    }

    public boolean hasPivotArrived() {
        return Math.abs(pivotSetpoint - pivotInternalEncoder.getPosition()) < 1.5;
    }

    public void setPivotAngle(Rotation2d rotation2d) {
        pivotSetpoint = rotation2d.getDegrees();
    }

    private void configureCANCoder(CANCoder encoder) {
        encoder.configFactoryDefault();
    }

    private void configureFlywheelMotor(CANSparkFlex motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    }

    private void configureSRXMotor(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
    }

    private void configurePivotMotor(CANSparkFlex motor) {
        configureFlywheelMotor(motor);

        pivotMotor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);
        pivotMotor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, PIVOT_CONSTRAINT_DEGREES);
        pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    private boolean doesSeeNote() {
        return !beamBreaker.get();
    }

    public void setPitchVoltage(Measure<Voltage> voltage) {
        pivotMotor.setVoltage(voltage.in(Volts));
    }

    public void logPitch(SysIdRoutineLog log) {
        log.motor("pivot")
            .voltage(
                m_appliedVoltage.mut_replace(
                    pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage(), Volts))
            .angularPosition(m_angle.mut_replace(pivotEncoder.getAbsolutePosition(), Degrees))
            .angularVelocity(
                m_velocity.mut_replace(pivotEncoder.getVelocity(), DegreesPerSecond));
    }
}
