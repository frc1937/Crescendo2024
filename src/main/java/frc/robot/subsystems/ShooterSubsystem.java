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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShootingConstants.FLYWHEEL_FF;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_LEFT_ID;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_MAX_RPM;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_P;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_RIGHT_ID;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_VELOCITY_TOLERANCE;
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

public class ShooterSubsystem extends SubsystemBase {
    private final DigitalInput beamBreaker = new DigitalInput(0);
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(KICKER_ID);
    private final CANSparkFlex flywheelMaster = new CANSparkFlex(FLYWHEEL_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex pivotMotor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder flywheelEncoder = flywheelMaster.getEncoder(SparkRelativeEncoder.Type.kNoSensor, 7168);
    private final RelativeEncoder pivotInternalEncoder = pivotMotor.getEncoder(SparkRelativeEncoder.Type.kNoSensor, 7168);
    private final CANCoder pivotEncoder = new CANCoder(PIVOT_CAN_CODER);
    private final SparkPIDController pitchController;
    private final SparkPIDController flywheelsController;
    private double pivotSetpoint = 0;
    private double targetFlywheelVelocity = 0;

    public ShooterSubsystem() {
        configureSRXMotor(kickerMotor);

        flywheelsController = flywheelMaster.getPIDController();
        configureFlywheelMotor(flywheelMaster);

        CANSparkFlex flywheelSlave = new CANSparkFlex(FLYWHEEL_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
        configureFlywheelMotor(flywheelSlave);

        configurePivotMotor(pivotMotor);

        configureCanCoder(pivotEncoder);

        flywheelSlave.follow(flywheelMaster, true);

        pitchController = pivotMotor.getPIDController();
        setupPivotController();
    }

    @Override
    public void periodic() {
        double currentAngle = -(pivotEncoder.getPosition() - PIVOT_ENCODER_OFFSET);
        pivotInternalEncoder.setPosition(currentAngle);

        /* FOR DEBUGGING, REMOVE */
        SmartDashboard.putNumber("Velocity", flywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Target Velocity", targetFlywheelVelocity);
        SmartDashboard.putNumber("Current angle", currentAngle);
        SmartDashboard.putNumber("Pivot setpoint", pivotSetpoint);
        SmartDashboard.putBoolean("Does see note", doesSeeNote());

        if(pivotSetpoint > 80) {
            pitchController.setReference(pivotSetpoint, ControlType.kPosition, 2);
        } else if (pivotSetpoint >= currentAngle) {
            pitchController.setReference(pivotSetpoint, CANSparkBase.ControlType.kPosition, 0);
        } else if (pivotSetpoint < currentAngle) {
            pitchController.setReference(pivotSetpoint, CANSparkBase.ControlType.kPosition, 1);
        }

        SmartDashboard.putNumber("Flywheel RPM", flywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Flywheel ANGLE", flywheelEncoder.getPosition());
    }

    public boolean doesSeeNote() {
        return !beamBreaker.get();
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
        return Math.abs(pivotSetpoint - pivotInternalEncoder.getPosition()) < 1.5;
    }

    public void setPivotAngle(Rotation2d rotation2d) {
        pivotSetpoint = rotation2d.getDegrees();
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

        pivotMotor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);
        pivotMotor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, PIVOT_CONSTRAINT_DEGREES);
        pivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    private void setupPivotController() {
        pitchController.setP(PIVOT_UP_P, 0);
        pitchController.setFF(PIVOT_UP_FF, 0);

        pitchController.setP(PIVOT_DOWN_P, 1);
        pitchController.setFF(PIVOT_DOWN_FF, 1);

        pitchController.setP(PIVOT_HIGH_P, 2);
        pitchController.setD(PIVOT_HIGH_D, 2);
        pitchController.setFF(PIVOT_HIGH_FF, 2);

        pitchController.setOutputRange(PIVOT_RANGE_MIN, PIVOT_RANGE_MAX);

        pivotInternalEncoder.setPosition(pivotEncoder.getAbsolutePosition());
        pivotEncoder.setPosition(pivotEncoder.getAbsolutePosition());
    }
}
