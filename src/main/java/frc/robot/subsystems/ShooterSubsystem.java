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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShootingConstants.FLYWHEEL_LEFT_ID;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_RIGHT_ID;
import static frc.robot.Constants.ShootingConstants.KICKER_ID;
import static frc.robot.Constants.ShootingConstants.MAXIMUM_OCCLUDED_PITCH;
import static frc.robot.Constants.ShootingConstants.MINIMUM_OCCLUDED_PITCH;
import static frc.robot.Constants.ShootingConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DEGREES;
import static frc.robot.Constants.ShootingConstants.PIVOT_CONSTRAINT_DIRECTION;
import static frc.robot.Constants.ShootingConstants.PIVOT_DOWN_FF;
import static frc.robot.Constants.ShootingConstants.PIVOT_DOWN_P;
import static frc.robot.Constants.ShootingConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.ShootingConstants.PIVOT_ID;
import static frc.robot.Constants.ShootingConstants.PIVOT_RANGE_MAX;
import static frc.robot.Constants.ShootingConstants.PIVOT_RANGE_MIN;
import static frc.robot.Constants.ShootingConstants.PIVOT_UP_FF;
import static frc.robot.Constants.ShootingConstants.PIVOT_UP_P;

public class ShooterSubsystem extends SubsystemBase {
    private final DigitalInput beamBreaker = new DigitalInput(0);
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(KICKER_ID);
    private final CANSparkFlex flywheelMaster = new CANSparkFlex(FLYWHEEL_LEFT_ID, CANSparkLowLevel.MotorType.kBrushless),
            flywheelSlave = new CANSparkFlex(FLYWHEEL_RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless),
            pivotMotor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder flywheelEncoder = flywheelMaster.getEncoder();
    private final CANCoder pivotEncoder = new CANCoder(PIVOT_CAN_CODER);
    private final SparkPIDController pivotController;
    private double pivotSetpoint = 0;

//    private SysIdRoutine sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
//            new SysIdRoutineLog("f").motor("bozo")
//                    .voltage(Units.Volts.of(pivotMotor.get()))
//
//                        ));
//
//    private void updateLog(SysIdRoutineLog log, String motorName, CANSparkFlex sparkFlex, CANCoder canCoder) {
//        log.motor(motorName)
//                .voltage(Volts.of(sparkFlex.getBusVoltage()))
//                .angularPosition(Units.Degrees.of(-(canCoder.getPosition() - PIVOT_ENCODER_OFFSET)))
//                .angularVelocity(Units.DegreesPerSecond.of(canCoder.getVelocity()))
//    }

    public ShooterSubsystem() {
        configureSRXMotor(kickerMotor);

        configureSparkMotor(flywheelMaster);
        configureSparkMotor(flywheelSlave);

        configurePivotMotor(pivotMotor);

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
        SmartDashboard.putNumber("AbsolutePosition", pivotEncoder.getPosition());
        SmartDashboard.putNumber("CurrentPosition ", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("CurrentVelocity ", pivotMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("CurrentAngle", currentAngle);

        if(pivotSetpoint >= currentAngle) {
            pivotController.setReference(pivotSetpoint, CANSparkBase.ControlType.kPosition, 0);
        } else {
            pivotController.setReference(pivotSetpoint, CANSparkBase.ControlType.kPosition, 1);
        }
//
        SmartDashboard.putNumber("Flywheel RPM", flywheelEncoder.getVelocity());
        SmartDashboard.putNumber("Flywheel ANGLE", flywheelEncoder.getPosition());
    }

    public boolean isOccluded() {
        double pitch = pivotMotor.getEncoder().getPosition();
        return pitch > MINIMUM_OCCLUDED_PITCH && pitch < MAXIMUM_OCCLUDED_PITCH;
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

    public void setFlywheelSpeed(double speed) {
        flywheelMaster.set(speed);
    }

    public void stopFlywheels() {
        flywheelMaster.stopMotor();
    }

    public boolean areFlywheelsReady(double rpm) {
        return Math.abs(flywheelEncoder.getVelocity()) >= rpm;
    }

    public boolean hasPivotArrived() {
        return Math.abs(pivotSetpoint - pivotMotor.getEncoder().getPosition()) < 3;
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

    private void configurePivotMotor(CANSparkFlex motor) {
        configureSparkMotor(motor);

        pivotMotor.enableSoftLimit(PIVOT_CONSTRAINT_DIRECTION, true);
        pivotMotor.setSoftLimit(PIVOT_CONSTRAINT_DIRECTION, PIVOT_CONSTRAINT_DEGREES);
    }

    private void setupPivotController() {
        pivotController.setP(PIVOT_UP_P, 0);
        pivotController.setFF(PIVOT_UP_FF, 0);

        pivotController.setP(PIVOT_DOWN_P, 1);
        pivotController.setFF(PIVOT_DOWN_FF, 1);

        pivotController.setOutputRange(PIVOT_RANGE_MIN, PIVOT_RANGE_MAX);

        pivotMotor.getEncoder().setPosition(pivotEncoder.getAbsolutePosition());
        pivotEncoder.setPosition(pivotEncoder.getAbsolutePosition());
    }
}
