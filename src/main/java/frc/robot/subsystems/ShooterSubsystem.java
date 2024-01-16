package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(Constants.ShooterConstants.kickerID);
    private final CANSparkFlex pivotMotor = new CANSparkFlex(Constants.ShooterConstants.pivotID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex flywheelMaster = new CANSparkFlex(Constants.ShooterConstants.flywheelLeftID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkFlex flywheelSlave = new CANSparkFlex(Constants.ShooterConstants.flywheelRightID, CANSparkLowLevel.MotorType.kBrushless);

    private final RelativeEncoder flywheelEncoder = flywheelMaster.getEncoder();
    private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    private final PIDController pivotPIDController;
    private final PIDController flywheelPIDController;

    private final static ShooterSubsystem INSTANCE = new ShooterSubsystem();

    public static ShooterSubsystem getInstance() {
        return INSTANCE;
    }

    public ShooterSubsystem() { //todo: calibrate PID values
        pivotPIDController = new PIDController(0, 0, 0);
        pivotPIDController.enableContinuousInput(-180, 180);

        flywheelPIDController = new PIDController(0, 0, 0);
        flywheelPIDController.enableContinuousInput(-180, 180);

        configureTalonMotor(kickerMotor);

        configureSparkMotor(flywheelSlave);
        configureSparkMotor(flywheelMaster);

        flywheelSlave.follow(flywheelMaster);
    }

    //todo: necessary configurations, no idea what they are
    private void configureTalonMotor(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
    }

    private void configureSparkMotor(CANSparkFlex motor) {
        motor.restoreFactoryDefaults();
    }

    /**
     * @param angle in degrees, 0 being lowest point
     */
    public void rotateFlywheel(Rotation2d angle) {
        Rotation2d currentRotations = Rotation2d.fromRotations(flywheelEncoder.getPosition());
        double voltageOutput = flywheelPIDController.calculate(currentRotations.getDegrees(), angle.getDegrees());

        flywheelMaster.setVoltage(voltageOutput);
    }


    /**
     * @param angle in degrees, 0 being lowest point
     */
    public void rotatePivot(Rotation2d angle) {
        Rotation2d currentRotations = Rotation2d.fromRotations(pivotEncoder.getPosition());
        double voltageOutput = pivotPIDController.calculate(currentRotations.getDegrees(), angle.getDegrees());

        pivotMotor.setVoltage(voltageOutput);
    }


    public void enableKicker(double value) {
        kickerMotor.setVoltage(value);
    }

    public void stopKicker() {
        kickerMotor.stopMotor();
    }
}
