package frc.robot;

import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil.Usage;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {

    // Number values
    public int moduleNumber;
    private final Rotation2d angleOffset;
    private Rotation2d lastAngle;

    // Motor and encoder values
    private final RelativeEncoder integratedAngleEncoder;
    private final CANSparkMax mAngleMotor;
    private final TalonFX driveMotor;
    private final CANCoder angleEncoder;

    // Speed control variables
    private final SparkPIDController angleController;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

    /**
     * Every module of the swerve needs to be defined, here we define it.
     *
     * @param moduleNumber    Is the number of the module (1 - front left for example)
     * @param moduleConstants Are the constants needed for module.
     */
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.WHEEL_CIRCUMFERENCE, Constants.Swerve.DRIVE_GEAR_RATIO);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01))
                        ? lastAngle
                        : desiredState.angle;
        angleController.setReference(angle.getDegrees(), com.revrobotics.CANSparkBase.ControlType.kPosition);

        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        mAngleMotor.restoreFactoryDefaults();

        CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);

        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_CONTINUOUS_CURRENT_LIMIT);

        mAngleMotor.setInverted(Constants.Swerve.ANGLE_INVERT);
        mAngleMotor.setIdleMode(Constants.Swerve.ANGLE_NEUTRAL_MODE);

        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.ANGLE_CONVERSION_FACTOR);
        angleController.setP(Constants.Swerve.ANGLE_KP);
        angleController.setI(Constants.Swerve.ANGLE_KI);
        angleController.setD(Constants.Swerve.ANGLE_KD);
        angleController.setFF(Constants.Swerve.ANGLE_KFF);

        mAngleMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();

        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
        driveMotor.setNeutralMode(Constants.Swerve.DRIVE_NEUTRAL_MODE);
        driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), Constants.Swerve.WHEEL_CIRCUMFERENCE, Constants.Swerve.DRIVE_GEAR_RATIO),
                getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.Swerve.WHEEL_CIRCUMFERENCE, Constants.Swerve.DRIVE_GEAR_RATIO),
                getAngle()
        );
    }
}