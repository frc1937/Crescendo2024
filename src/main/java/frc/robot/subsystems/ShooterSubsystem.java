package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShootingConstants.FLYWHEEL_LEFT_ID;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_MINIMUM_READY_SPEED;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_RIGHT_ID;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_SPEED;
import static frc.robot.Constants.ShootingConstants.KICKER_ID;
import static frc.robot.Constants.ShootingConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.ShootingConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.ShootingConstants.PIVOT_FF;
import static frc.robot.Constants.ShootingConstants.PIVOT_ID;
import static frc.robot.Constants.ShootingConstants.PIVOT_P;
import static frc.robot.Constants.ShootingConstants.PIVOT_RANGE_MAX;
import static frc.robot.Constants.ShootingConstants.PIVOT_RANGE_MIN;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(KICKER_ID);
    private final CANSparkFlex flywheelMaster = new CANSparkFlex(FLYWHEEL_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless),
            flywheelSlave = new CANSparkFlex(FLYWHEEL_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless),
            pivotMotor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder flywheelEncoder = flywheelMaster.getEncoder();
    private final CANCoder pivotEncoder = new CANCoder(PIVOT_CAN_CODER);
    private final SparkPIDController pivotController;
    private double pivotSetpoint = 0;

    public ShooterSubsystem() {
        configureSRXMotor(kickerMotor);

        configureSparkMotor(flywheelMaster);
        configureSparkMotor(flywheelSlave);
        configureSparkMotor(pivotMotor);

        configureCanCoder(pivotEncoder);

        flywheelSlave.follow(flywheelMaster, true);

        pivotController = pivotMotor.getPIDController();
        setupPivotController();
    }

    @Override
    public void periodic() {
        double currentAngle = -(pivotEncoder.getPosition() - PIVOT_ENCODER_OFFSET);
        pivotMotor.getEncoder().setPosition(currentAngle);

        /* FOR DEBUGGING, REMOVE */
        SmartDashboard.putNumber("CurrentAngle ", currentAngle);
        SmartDashboard.putNumber("AbsolutePosition", pivotEncoder.getPosition());
        SmartDashboard.putNumber("CurrentPosition ", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("CurrentVelocity ", pivotMotor.getEncoder().getVelocity());

        pivotController.setReference(pivotSetpoint, CANSparkBase.ControlType.kPosition);
    }

    public void startKicker(double speed) {
        kickerMotor.set(ControlMode.PercentOutput, speed);
    }

    public void startFlywheels() {
        flywheelMaster.set(FLYWHEEL_SPEED);
    }

    public void setFlywheelSpeed(double speed) {
        flywheelMaster.set(speed);
    }

    public void stopFlywheels() {
        flywheelMaster.stopMotor();
    }

    public boolean areFlywheelsReady() {
        return Math.abs(flywheelEncoder.getVelocity()) > FLYWHEEL_MINIMUM_READY_SPEED;
    }

    public boolean hasPivotArrived() {
        return Math.abs(pivotSetpoint - pivotMotor.getEncoder().getPosition()) < 0.25;
    }

    public void setPivotAngle(Rotation2d rotation2d) {
        pivotSetpoint = rotation2d.getDegrees();
    }


    private void configureCanCoder(CANCoder encoder) {
        encoder.configFactoryDefault();
    }

    private void configureSparkMotor(CANSparkFlex motor) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }

    private void configureSRXMotor(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
    }

    private void setupPivotController() {
        pivotController.setP(PIVOT_P);
        pivotController.setFF(PIVOT_FF);
        pivotController.setOutputRange(PIVOT_RANGE_MIN, PIVOT_RANGE_MAX);

        pivotMotor.getEncoder().setPosition(pivotEncoder.getAbsolutePosition());
        pivotEncoder.setPosition(pivotEncoder.getAbsolutePosition());
    }
}
