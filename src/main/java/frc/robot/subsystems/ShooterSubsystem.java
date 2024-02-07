package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.FLYWHEEL_LEFT_ID;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_MINIMUM_READY_SPEED;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL_RIGHT_ID;
import static frc.robot.Constants.ShooterConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.ShooterConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.ShooterConstants.PIVOT_FF;
import static frc.robot.Constants.ShooterConstants.PIVOT_ID;
import static frc.robot.Constants.ShooterConstants.PIVOT_P;
import static frc.robot.Constants.ShooterConstants.PIVOT_RANGE_MAX;
import static frc.robot.Constants.ShooterConstants.PIVOT_RANGE_MIN;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkFlex flywheelMaster = new CANSparkFlex(FLYWHEEL_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex flywheelSlave = new CANSparkFlex(FLYWHEEL_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex pivotMotor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder flywheelEncoder = flywheelMaster.getEncoder();

    private final CANCoder pivotEncoder = new CANCoder(PIVOT_CAN_CODER);
    private final SparkPIDController pivotController;
    private double pivotSetpoint = 0;

    public ShooterSubsystem() {
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

    private void setupPivotController() {
        pivotController.setP(PIVOT_P);
        pivotController.setFF(PIVOT_FF);
        pivotController.setOutputRange(PIVOT_RANGE_MIN, PIVOT_RANGE_MAX);

        pivotMotor.getEncoder().setPosition(pivotEncoder.getAbsolutePosition());
        pivotEncoder.setPosition(pivotEncoder.getAbsolutePosition());
    }
}
