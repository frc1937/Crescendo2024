package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(Constants.ShooterConstants.KICKER_ID);
    private final CANSparkFlex pivotMotor = new CANSparkFlex(Constants.ShooterConstants.PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex flywheelMaster = new CANSparkFlex(Constants.ShooterConstants.FLYWHEEL_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex flywheelSlave = new CANSparkFlex(Constants.ShooterConstants.FLYWHEEL_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder flywheelEncoder = flywheelMaster.getEncoder();
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    private final PIDController pivotPIDController;
    private final PIDController flywheelPIDController;

    public Shooter() { //todo: calibrate PID values
        pivotPIDController = new PIDController(PIVOT_KP, PIVOT_KI, PIVOT_KD);
        pivotPIDController.enableContinuousInput(-180, 180);

        flywheelPIDController = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheelPIDController.enableContinuousInput(-180, 180);

        configureTalonMotor(kickerMotor);

        configureSparkMotor(flywheelSlave);
        configureSparkMotor(flywheelMaster);

        flywheelSlave.setInverted(true);
        flywheelSlave.follow(flywheelMaster);
    }

    @Override
    public void periodic() {
        Rotation2d flywheelRotation = Rotation2d.fromRotations(flywheelEncoder.getPosition());
        Rotation2d pivotRotation = Rotation2d.fromRotations(pivotEncoder.getPosition());

        double flywheelOutput = flywheelPIDController.calculate(flywheelRotation.getDegrees());
        double pivotOutput = pivotPIDController.calculate(pivotRotation.getDegrees());

        pivotMotor.setVoltage(pivotOutput);
        flywheelMaster.setVoltage(flywheelOutput);
    }

    /**
     * @param angle in rotations, turned into degress. 0 being lowest point
     */
    public void rotateFlywheel(Rotation2d angle) {
        flywheelPIDController.setSetpoint(angle.getDegrees());
    }


    /**
     * @param angle in rotations, turned into degress. 0 being lowest point
     */
    public void rotatePivot(Rotation2d angle) {
        pivotPIDController.setSetpoint(angle.getDegrees());
    }


    public void setKickerVoltage(double value) {
        kickerMotor.setVoltage(value);
    }

    public void stopKicker() {
        kickerMotor.stopMotor();
    }

    public void stopFlywheel() {
        flywheelMaster.stopMotor();
    }


    //todo: necessary configurations, no idea what they are
    private void configureTalonMotor(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
    }

    private void configureSparkMotor(CANSparkFlex motor) {
        motor.restoreFactoryDefaults();
    }
}
