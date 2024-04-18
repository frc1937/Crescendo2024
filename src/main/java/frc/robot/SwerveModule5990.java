package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.lib.math.Conversions;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.constants.Constants.ODOMETRY_FREQUENCY_HERTZ;
import static frc.robot.constants.SwerveConstants.*;

public class SwerveModule5990 {
    public final SwerveModuleConstants swerveModuleConstants;

    private final TalonFX driveMotor;
    private final CANSparkMax steerMotor;
    private final SparkPIDController steerController;
    private final RelativeEncoder steerRelativeEncoder;
    private final CANcoder steerEncoder;
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

    private Rotation2d lastAngle;

    /**
     * Rotations
     */
    private StatusSignal<Double> steerPositionSignal;
    /**
     * The position of the drive motor in rotations
     */
    private StatusSignal<Double> drivePositionSignal;
    /**
     * Rotations per second
     */
    private StatusSignal<Double> steerVelocitySignal, driveVelocitySignal;

    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0).withSlot(0);
    //You will control driveMotor velocity using feedforward to get the voltage.

    public SwerveModule5990(SwerveModuleConstants constants) {
        this.swerveModuleConstants = constants;

        this.driveMotor = new TalonFX(swerveModuleConstants.driveMotorID);
        this.steerEncoder = new CANcoder(swerveModuleConstants.cancoderID);
        this.steerMotor = new CANSparkMax(swerveModuleConstants.steerMotorID, CANSparkLowLevel.MotorType.kBrushless);

        this.steerController = steerMotor.getPIDController();
        this.steerRelativeEncoder = steerMotor.getEncoder();

        configureSteerEncoder();
        configureSteerMotor();
        configureDriveMotor();
    }

    public void setTargetState(SwerveModuleState targetState, boolean closedLoop) {
        targetState = CTREModuleState.optimize(targetState, getCurrentAngle());

        setTargetVelocity(targetState, closedLoop);
        setTargetAngle(targetState);
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                Conversions.rotationsPerSecondToMetersPerSecond(driveVelocitySignal.refresh().getValue()), //todo: This MIGHT be correct
                getCurrentAngle()
        );
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(drivePositionSignal.refresh().getValue()), //TODO: I doubt this actually returns metres.
                getCurrentAngle()
        );
    }


    public Rotation2d getCurrentAngle() {
        return new Rotation2d(steerPositionSignal.refresh().getValue());
    }

    public void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    /**
     * Sets the target velocity for the module.
     */
    private void setTargetVelocity(SwerveModuleState targetState, boolean closedLoop) {
        Measure<Velocity<Distance>> targetVelocity = CTREModuleState.reduceSkew(MetersPerSecond.of(targetState.speedMetersPerSecond), targetState.angle, getCurrentAngle());

        if (closedLoop)
            setTargetClosedLoopVelocity(targetVelocity);
        else
            setTargetOpenLoopVelocity(targetVelocity);
    }

    private void setTargetOpenLoopVelocity(Measure<Velocity<Distance>> targetVelocity) {
        // Scale metres per second to [-1, 1]
        double power = targetVelocity.in(MetersPerSecond) / MAX_SPEED;
        // Scale [-1, 1] to [-12, 12]
        double voltage = Conversions.compensatedPowerToVoltage(power, VOLTAGE_COMPENSATION_SATURATION);

        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    private void setTargetClosedLoopVelocity(Measure<Velocity<Distance>> targetVelocity) {
        if (targetVelocity.in(MetersPerSecond) <= 0.01) {
            return;
        }

        driveMotor.setControl(driveVelocityRequest
                .withVelocity(targetVelocity.in(MetersPerSecond))
//                .withFeedForward(driveFeedforward.calculate(targetVelocity.in(MetersPerSecond))) todo test
        );
    }

    private void setTargetAngle(SwerveModuleState targetState) {
        Rotation2d angle =
                (Math.abs(targetState.speedMetersPerSecond) <= SWERVE_IN_PLACE_DRIVE_MPS)
                        ? lastAngle
                        : targetState.angle;

        steerController.setReference(angle.getDegrees(), CANSparkBase.ControlType.kPosition);

        lastAngle = angle;
    }

    //Configure canCoder, steer motor, and drive motor
    private void configureSteerMotor() {
        steerMotor.restoreFactoryDefaults();

        CANSparkMaxUtil.setCANSparkMaxBusUsage(steerMotor, CANSparkMaxUtil.Usage.kPositionOnly);

        steerMotor.setSmartCurrentLimit(ANGLE_CONTINUOUS_CURRENT_LIMIT);

        steerMotor.setInverted(ANGLE_INVERT);
        steerMotor.setIdleMode(ANGLE_NEUTRAL_MODE);

        steerRelativeEncoder.setPositionConversionFactor(ANGLE_CONVERSION_FACTOR);

        steerController.setP(ANGLE_KP);
        steerController.setI(ANGLE_KI);
        steerController.setD(ANGLE_KD);
        steerController.setFF(ANGLE_KFF);

        steerMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION_SATURATION);
        steerMotor.burnFlash();

        steerRelativeEncoder.setPosition(steerPositionSignal.refresh().getValue()); //This might be in the wrong units
        //This resets the relative encoder position to the steer pos. However, idk if this is correct usage.
    }

    private void configureSteerEncoder() {
        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();

        swerveCanCoderConfig.MagnetSensor.MagnetOffset = swerveModuleConstants.angleOffset.getRotations();
        swerveCanCoderConfig.MagnetSensor.SensorDirection = CAN_CODER_INVERT;
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1; //todo:
        //TODO XXX WARNING THIS HAS CHANGED FROM 0 - 360 TO 0 - 1. CODE MIGHT STILL USE OLD VALUES. PLEASE CHECK!

        applyConfig(steerEncoder, swerveCanCoderConfig);

        steerPositionSignal = steerEncoder.getPosition().clone();
        steerVelocitySignal = steerEncoder.getVelocity().clone();

        steerPositionSignal.setUpdateFrequency(ODOMETRY_FREQUENCY_HERTZ);
        steerVelocitySignal.setUpdateFrequency(ODOMETRY_FREQUENCY_HERTZ);

        steerEncoder.optimizeBusUtilization();
    }

    private void configureDriveMotor() {
        TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();

        swerveDriveFXConfig.Audio.BeepOnConfig = true;
        swerveDriveFXConfig.Audio.BeepOnBoot = true;

        swerveDriveFXConfig.MotorOutput.Inverted = DRIVE_MOTOR_INVERT;
        swerveDriveFXConfig.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = DRIVE_ENABLE_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_CONTINUOUS_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = DRIVE_PEAK_CURRENT_LIMIT;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = DRIVE_PEAK_CURRENT_DURATION;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = DRIVE_KP;
        swerveDriveFXConfig.Slot0.kI = DRIVE_KI;
        swerveDriveFXConfig.Slot0.kD = DRIVE_KD;
        swerveDriveFXConfig.Slot0.kV = DRIVE_KV;
        swerveDriveFXConfig.Slot0.kA = DRIVE_KA;
        swerveDriveFXConfig.Slot0.kS = DRIVE_KS;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = OPEN_LOOP_RAMP;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = CLOSED_LOOP_RAMP;

        applyConfig(driveMotor, swerveDriveFXConfig);

        drivePositionSignal = driveMotor.getPosition().clone();
        driveVelocitySignal = driveMotor.getVelocity().clone();

        drivePositionSignal.setUpdateFrequency(ODOMETRY_FREQUENCY_HERTZ);
        driveVelocitySignal.setUpdateFrequency(ODOMETRY_FREQUENCY_HERTZ);

        driveMotor.optimizeBusUtilization();
    }

    private void applyConfig(Object device, Object config) {
        int counter = 10;
        StatusCode statusCode = null;

        while (statusCode != StatusCode.OK && counter > 0) {
            if (device instanceof TalonFX talonFX && config instanceof TalonFXConfiguration swerveDriveFXConfig) {
                statusCode = talonFX.getConfigurator().apply(swerveDriveFXConfig);
            }

            if (device instanceof CANcoder canCoder && config instanceof CANcoderConfiguration swerveCanCoderConfig) {
                statusCode = canCoder.getConfigurator().apply(swerveCanCoderConfig);
            }

            counter--;
        }
    }
}
