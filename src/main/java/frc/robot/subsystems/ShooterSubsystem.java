package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(Constants.ShooterConstants.KICKER_ID);
    private final CANSparkFlex pivotMotor = new CANSparkFlex(Constants.ShooterConstants.PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex flywheelMaster = new CANSparkFlex(FLYWHEEL_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex flywheelSlave = new CANSparkFlex(FLYWHEEL_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder flywheelEncoder = flywheelMaster.getEncoder();
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    private final PIDController pivotPIDController;
    private final SparkPIDController flywheelPIDController = flywheelMaster.getPIDController();

    public ShooterSubsystem() {
        pivotPIDController = new PIDController(PIVOT_KP, PIVOT_KI, PIVOT_KD);
        pivotPIDController.enableContinuousInput(-180, 180);

        flywheelPIDController.setP(FLYWHEEL_KP);
        flywheelPIDController.setI(FLYWHEEL_KI);
        flywheelPIDController.setD(FLYWHEEL_KD);
        flywheelPIDController.setOutputRange(FLYWHEEL_KMIN_OUTPUT, FLYWHEEL_KMAX_OUTPUT);

        configureTalonMotor(kickerMotor);

        flywheelSlave.follow(flywheelMaster, true);

        configureSparkMotor(flywheelSlave);
        configureSparkMotor(flywheelMaster);
    }

    @Override
    public void periodic() {
        Rotation2d pivotRotation = Rotation2d.fromRotations(pivotEncoder.getPosition());
        double pivotOutput = pivotPIDController.calculate(pivotRotation.getDegrees());

        pivotMotor.setVoltage(pivotOutput);
    }

    public void startFlywheels() {
        double targetVelocity = 0.8;
        flywheelPIDController.setReference(targetVelocity, CANSparkBase.ControlType.kVelocity);
        //This should automatically apply voltage to the motor. Not sure tho, should check.
    }

    public boolean areFlywheelsReady() {
        return Math.abs(flywheelEncoder.getVelocity()) > FLYWHEEL_MINIMUM_READY_SPEED;
    }


    /**
     * @param angle in rotations, turned into degress. 0 being lowest point
     */
    public void setPivotSetpoint(Rotation2d angle) {
        pivotPIDController.setSetpoint(angle.getDegrees());
    }


    public void setKickerVoltage(double value) {
        //   kickerMotor.setVoltage(value);
    }

    public void stopKicker() {
        // kickerMotor.stopMotor();
    }

    public void stopFlywheel() {
        flywheelMaster.stopMotor();
    }


    private void configureTalonMotor(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
    }

    private void configureSparkMotor(CANSparkFlex motor) {
        motor.restoreFactoryDefaults();
    }
}
