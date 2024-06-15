package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.math.Conversions;
import frc.lib.motor.*;
import frc.lib.util.CTREModuleState;

import static frc.lib.math.Conversions.metersPerSecondToRotationsPerSecond;
import static frc.lib.util.CTREUtil.applyConfig;
import static frc.robot.Constants.ODOMETRY_FREQUENCY_HERTZ;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class SwerveModule5990 {
    private static final double WHEEL_DIAMETER = WHEEL_CIRCUMFERENCE / Math.PI;

    public final SwerveConstants.SwerveModuleConstants swerveModuleConstants;

    private Motor driveMotor;
    private Motor steerMotor;

    /**
     * Rotations
     */
    private StatusSignal<Double> steerPositionSignal;

    private SwerveModuleState targetState;

    public SwerveModule5990(SwerveConstants.SwerveModuleConstants constants) {
        this.swerveModuleConstants = constants;

        configureDriveMotor();
        configureSteerMotor();
        configureSteerAbsoluteEncoder();
    }

    public void setTargetState(final SwerveModuleState targetState, boolean closedLoop) {
        this.targetState = CTREModuleState.optimize(targetState, getCurrentAngle());

        driveToTargetVelocity(closedLoop);
        driveToTargetAngle();
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                Conversions.rotationsPerSecondToMetersPerSecond(driveMotor.getSystemVelocity(), WHEEL_DIAMETER),
                getCurrentAngle()
        );
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(driveMotor.getSystemPosition(), WHEEL_DIAMETER),
                getCurrentAngle()
        );
    }

    public double getWheelDistanceTraveledRadians() {
        return Units.rotationsToRadians(driveMotor.getMotorPosition());
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(steerPositionSignal.refresh().getValue() - swerveModuleConstants.angleOffset());
    }

    public void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    public void periodic() {
        steerMotor.setMotorPosition(getCurrentAngle().getRotations());
        steerMotor.setP(ANGLE_KP.get(), 0);
    }

    /**
     * Sets the target velocity for the module.
     */
    private void driveToTargetVelocity(boolean closedLoop) {
        double targetVelocityMPS = CTREModuleState.reduceSkew(targetState.speedMetersPerSecond, targetState.angle, getCurrentAngle());

        if (closedLoop)
            setTargetClosedLoopVelocity(targetVelocityMPS);
        else
            setTargetOpenLoopVelocity(targetVelocityMPS);
    }

    private void setTargetOpenLoopVelocity(double targetVelocityMPS) {
        double powerPercentage = targetVelocityMPS / MAX_SPEED_MPS;
        driveMotor.setOutput(MotorProperties.ControlMode.PERCENTAGE_OUTPUT, powerPercentage);
    }

    private void setTargetClosedLoopVelocity(double targetVelocityMPS) {
        if (targetVelocityMPS <= SWERVE_IN_PLACE_DRIVE_MPS) {
            return;
        }

        double targetVelocityRPS = metersPerSecondToRotationsPerSecond(targetVelocityMPS, WHEEL_DIAMETER);
        driveMotor.setOutput(MotorProperties.ControlMode.VELOCITY, targetVelocityRPS);
    }

    private void driveToTargetAngle() {
        steerMotor.setOutput(MotorProperties.ControlMode.POSITION, targetState.angle.getRotations());
    }

    private void configureSteerMotor() {
        steerMotor = new GenericSpark(swerveModuleConstants.steerMotorID(), CANSparkLowLevel.MotorType.kBrushless);

        MotorConfiguration steerConfiguration = new MotorConfiguration();

        steerConfiguration.supplyCurrentLimit = ANGLE_CURRENT_LIMIT;
        steerConfiguration.inverted = ANGLE_MOTOR_INVERT;
        steerConfiguration.idleMode = ANGLE_NEUTRAL_MODE;

        steerConfiguration.conversionFactor = ANGLE_GEAR_RATIO;// 7.0 / 150.0;

        steerMotor.setMotorPosition(getCurrentAngle().getRotations());

        steerMotor.configure(steerConfiguration);
    }

    private void configureSteerAbsoluteEncoder() {
        CANcoder steerAbsoluteEncoder = new CANcoder(swerveModuleConstants.canCoderID());

        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();

        swerveCanCoderConfig.MagnetSensor.SensorDirection = CAN_CODER_INVERT;
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        applyConfig(steerAbsoluteEncoder, swerveCanCoderConfig);
        steerAbsoluteEncoder.optimizeBusUtilization();

        steerPositionSignal = steerAbsoluteEncoder.getPosition().clone();
        steerPositionSignal.setUpdateFrequency(ODOMETRY_FREQUENCY_HERTZ);
    }

    private void configureDriveMotor() {
        driveMotor = new GenericTalonFX(swerveModuleConstants.driveMotorID());

        MotorConfiguration driveConfiguration = new MotorConfiguration();

        driveConfiguration.idleMode = DRIVE_NEUTRAL_MODE;
        driveConfiguration.inverted = DRIVE_MOTOR_INVERT;

        driveConfiguration.conversionFactor = DRIVE_GEAR_RATIO;

        /* Current Limiting */
        driveConfiguration.statorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;
        driveConfiguration.supplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT;

        /* PID-FF Config */
        driveConfiguration.slot0 = new MotorProperties.Slot(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KV, DRIVE_KA, DRIVE_KS, 0, null);

        /* Open and Closed Loop Ramping */
        driveConfiguration.dutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        driveConfiguration.dutyCycleCloseLoopRampPeriod = CLOSED_LOOP_RAMP;

        driveMotor.configure(driveConfiguration);
    }
}
