package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CTREModuleState;

import static frc.lib.math.Conversions.metersPerSecondToRotationsPerSecond;
import static frc.lib.util.CTREUtil.applyConfig;
import static frc.robot.Constants.ODOMETRY_FREQUENCY_HERTZ;
import static frc.robot.Constants.VOLTAGE_COMPENSATION_SATURATION;
import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_CURRENT_LIMIT;
import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_KD;
import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_KI;
import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_KP;
import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_MOTOR_INVERT;
import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_NEUTRAL_MODE;
import static frc.robot.subsystems.swerve.SwerveConstants.CAN_CODER_INVERT;
import static frc.robot.subsystems.swerve.SwerveConstants.CLOSED_LOOP_RAMP;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_GEAR_RATIO;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KA;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KD;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KI;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KP;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KS;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KV;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_MOTOR_INVERT;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_NEUTRAL_MODE;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_PEAK_CURRENT_DURATION;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_STATOR_CURRENT_LIMIT;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_SUPPLY_CURRENT_LIMIT;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve.SwerveConstants.OPEN_LOOP_RAMP;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_IN_PLACE_DRIVE_MPS;
import static frc.robot.subsystems.swerve.SwerveConstants.WHEEL_CIRCUMFERENCE;

public class SwerveModule5990 {
    private static final double WHEEL_DIAMETER = WHEEL_CIRCUMFERENCE / Math.PI;

    public final SwerveConstants.SwerveModuleConstants swerveModuleConstants;
    private final int moduleId;

    private TalonFX driveMotor;
    private CANSparkMax steerMotor;
    private SparkPIDController steerController;
    private RelativeEncoder steerRelativeEncoder;
    private CANcoder steerAbsoluteEncoder;

    private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0).withSlot(0);

    /**
     * Rotations
     */
    private StatusSignal<Double> steerPositionSignal;
    /**
     * Position is in rotations. Velocity is in rotations per second
     */
    private StatusSignal<Double> drivePositionSignal, driveVelocitySignal;

    private SwerveModuleState targetState;

    public SwerveModule5990(SwerveConstants.SwerveModuleConstants constants) {
        this.swerveModuleConstants = constants;
        this.moduleId = constants.moduleNumber();

        driveMotor = new TalonFX(swerveModuleConstants.driveMotorID());
        steerMotor = new CANSparkMax(swerveModuleConstants.steerMotorID(), CANSparkLowLevel.MotorType.kBrushless);
        steerAbsoluteEncoder = new CANcoder(swerveModuleConstants.canCoderID());

        configureDriveMotor();
        configureSteerMotor();
        configureSteerAbsoluteEncoder();

        steerRelativeEncoder = steerMotor.getEncoder();
        steerController = steerMotor.getPIDController();

        configureSteerController();
        configureSteerRelativeEncoder();
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
                Conversions.rotationsPerSecondToMetersPerSecond(driveVelocitySignal.refresh().getValue(), WHEEL_DIAMETER),
                getCurrentAngle()
        );
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(drivePositionSignal.refresh().getValue(), WHEEL_DIAMETER),
                getCurrentAngle()
        );
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(steerPositionSignal.refresh().getValue() - swerveModuleConstants.angleOffset());
    }

    public void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    public void periodic() {
        steerRelativeEncoder.setPosition(getCurrentAngle().getRotations());
        steerController.setP(ANGLE_KP.get());

        SmartDashboard.putNumber(moduleId + "steer-relative-pose-", steerRelativeEncoder.getPosition());
        SmartDashboard.putNumber(moduleId + "steer-absolute-pose-", getCurrentAngle().getRotations());
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
        driveMotor.setControl(driveDutyCycleRequest.withOutput(powerPercentage));
    }

    private void setTargetClosedLoopVelocity(double targetVelocityMPS) {
        if (targetVelocityMPS <= SWERVE_IN_PLACE_DRIVE_MPS) {
            return;
        }

        double targetVelocityRPS = metersPerSecondToRotationsPerSecond(targetVelocityMPS, WHEEL_DIAMETER);
        driveMotor.setControl(driveVelocityRequest.withVelocity(targetVelocityRPS));
    }

    private void driveToTargetAngle() {
        steerController.setReference(targetState.angle.getRotations(), CANSparkBase.ControlType.kPosition);
        SmartDashboard.putNumber(moduleId + " TargetStateAngle", targetState.angle.getDegrees());
    }

    private void configureSteerMotor() {
        steerMotor.restoreFactoryDefaults();

        steerMotor.setSmartCurrentLimit(ANGLE_CURRENT_LIMIT);
        steerMotor.setInverted(ANGLE_MOTOR_INVERT);
        steerMotor.setIdleMode(ANGLE_NEUTRAL_MODE);
        steerMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);

        CANSparkMaxUtil.setCANSparkBusUsage(steerMotor, CANSparkMaxUtil.Usage.kPositionOnly);
        steerMotor.burnFlash();
    }

    private void configureSteerAbsoluteEncoder() {
        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();

        swerveCanCoderConfig.MagnetSensor.SensorDirection = CAN_CODER_INVERT;
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        applyConfig(steerAbsoluteEncoder, swerveCanCoderConfig);
        steerAbsoluteEncoder.optimizeBusUtilization();

        steerPositionSignal = steerAbsoluteEncoder.getPosition().clone();
        steerPositionSignal.setUpdateFrequency(ODOMETRY_FREQUENCY_HERTZ);
    }

    private void configureDriveMotor() {
        TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();

        swerveDriveFXConfig.Audio.BeepOnBoot = false;

        swerveDriveFXConfig.MotorOutput.Inverted = DRIVE_MOTOR_INVERT;
        swerveDriveFXConfig.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;

        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable = DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = DRIVE_PEAK_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = DRIVE_PEAK_CURRENT_DURATION;

        /* PID-FF Config */
        swerveDriveFXConfig.Slot0.kP = DRIVE_KP;
        swerveDriveFXConfig.Slot0.kI = DRIVE_KI;
        swerveDriveFXConfig.Slot0.kD = DRIVE_KD;

        swerveDriveFXConfig.Slot0.kV = DRIVE_KV;
        swerveDriveFXConfig.Slot0.kA = DRIVE_KA;
        swerveDriveFXConfig.Slot0.kS = DRIVE_KS;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = CLOSED_LOOP_RAMP;

        applyConfig(driveMotor, swerveDriveFXConfig);
        driveMotor.optimizeBusUtilization();

        drivePositionSignal = driveMotor.getPosition().clone();
        driveVelocitySignal = driveMotor.getVelocity().clone();

        drivePositionSignal.setUpdateFrequency(ODOMETRY_FREQUENCY_HERTZ);
        driveVelocitySignal.setUpdateFrequency(ODOMETRY_FREQUENCY_HERTZ);
    }


    private void configureSteerController() {
        steerController.setP(ANGLE_KP.get());
        steerController.setI(ANGLE_KI);
        steerController.setD(ANGLE_KD);
    }

    private void configureSteerRelativeEncoder() {
        steerRelativeEncoder.setPosition(getCurrentAngle().getRotations());
        steerRelativeEncoder.setPositionConversionFactor(7.0 / 150.0);
    }
}
