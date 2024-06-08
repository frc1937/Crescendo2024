package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CTREModuleState;

import static frc.lib.math.Conversions.metersPerSecondToRotationsPerSecond;
import static frc.lib.util.CTREUtil.applyConfig;
import static frc.robot.Constants.ODOMETRY_FREQUENCY_HERTZ;
import static frc.robot.Constants.VOLTAGE_COMPENSATION_SATURATION;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class SwerveModule5990 {
    public final SwerveConstants.SwerveModuleConstants swerveModuleConstants;

    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final PIDController steerController = new PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD);
    private final CANcoder steerEncoder;

    private final double wheelDiameter = WHEEL_CIRCUMFERENCE / Math.PI;
    /**
     * Rotations
     */
    private StatusSignal<Double> steerPositionSignal;
    /**
     * The position of the drive motor in rotations
     */
    private StatusSignal<Double> drivePositionSignal;
    private StatusSignal<Double> driveVelocitySignal;

    private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0).withSlot(0);

    private SwerveModuleState targetState;

    public SwerveModule5990(SwerveConstants.SwerveModuleConstants constants) {
        this.swerveModuleConstants = constants;

        this.driveMotor = new TalonFX(swerveModuleConstants.driveMotorID());
        this.steerEncoder = new CANcoder(swerveModuleConstants.canCoderID());
        this.steerMotor = new CANSparkMax(swerveModuleConstants.steerMotorID(), CANSparkLowLevel.MotorType.kBrushless);

        steerController.setP(ANGLE_KP);
        steerController.setI(ANGLE_KI);
        steerController.setD(ANGLE_KD);

        configureSteerEncoder();
        configureDriveMotor();
        configureSteerMotor();
    }

    public void setTargetState(SwerveModuleState targetState, boolean closedLoop) {
        targetState = CTREModuleState.optimize(targetState, getCurrentAngle());

        setTargetVelocity(targetState, closedLoop);
        setTargetAngle(targetState);

        this.targetState = targetState;
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                Conversions.rotationsPerSecondToMetersPerSecond(driveVelocitySignal.refresh().getValue(), wheelDiameter),
                getCurrentAngle()
        );
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(drivePositionSignal.refresh().getValue(), wheelDiameter),
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

    /**
     * Sets the target velocity for the module.
     */
    private void setTargetVelocity(SwerveModuleState targetState, boolean closedLoop) {
        double targetVelocityMPS = CTREModuleState.reduceSkew(targetState.speedMetersPerSecond, targetState.angle, getCurrentAngle());

        if (closedLoop)
            setTargetClosedLoopVelocity(targetVelocityMPS);
        else
            setTargetOpenLoopVelocity(targetVelocityMPS);
    }

    private void setTargetOpenLoopVelocity(double targetVelocityMPS) {
        // Scale metres per second to [-1, 1]
        double power = targetVelocityMPS / MAX_SPEED_MPS;

        driveMotor.setControl(driveDutyCycleRequest.withOutput(power));
    }

    private void setTargetClosedLoopVelocity(double targetVelocityMPS) {
        if (targetVelocityMPS <= SWERVE_IN_PLACE_DRIVE_MPS) {
            return;
        }

        double targetVelocityRPS = metersPerSecondToRotationsPerSecond(targetVelocityMPS, wheelDiameter);
        driveMotor.setControl(driveVelocityRequest.withVelocity(targetVelocityRPS));
    }

    private void setTargetAngle(SwerveModuleState targetState) {
        steerMotor.setVoltage(steerController.calculate(getCurrentAngle().getDegrees(), targetState.angle.getDegrees()));
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

    private void configureSteerEncoder() {
        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();

        swerveCanCoderConfig.MagnetSensor.SensorDirection = CAN_CODER_INVERT;
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        applyConfig(steerEncoder, swerveCanCoderConfig);
        steerEncoder.optimizeBusUtilization();

        steerPositionSignal = steerEncoder.getPosition().clone();
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
}
