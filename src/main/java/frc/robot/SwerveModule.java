package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.lib.math.Conversions;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.Swerve.DRIVE_KP;
import static frc.robot.Constants.Swerve.SWERVE_IN_PLACE_DRIVE_MPS;

public class SwerveModule {
    public final int moduleNumber;
    private final Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private final RelativeEncoder integratedAngleEncoder;
    private final CANSparkMax mAngleMotor;
    private final TalonFX driveMotor;
    private final CANCoder angleEncoder;
    private final PIDController driveController;
    private final SparkPIDController angleController;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Swerve.DRIVE_KS, Swerve.DRIVE_KV, Swerve.DRIVE_KA);

    /**
     * Every module of the swerve needs to be defined, here we define it.
     *
     * @param moduleNumber    Is the number of the module (1 - front left for example)
     * @param moduleConstants Are the constants needed for module.
     */
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, CANSparkLowLevel.MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        configAngleMotor();

        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
        driveController = new PIDController(DRIVE_KP, 0, 0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean closedLoop) {
        // This is a custom optimize function, since default WPILib optimize assumes continuous
        // controller which CTRE and Rev onboard is not
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        setSpeed(MetersPerSecond.of(desiredState.speedMetersPerSecond), closedLoop);
        setAngle(desiredState);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        setDesiredState(desiredState, false);
    }

    private void setSpeed(Measure<Velocity<Distance>> speed, boolean closedLoop) {
        if (closedLoop) {
            if (Math.abs(speed.in(MetersPerSecond)) < 0.01) {
                driveMotor.set(ControlMode.PercentOutput, 0);
            } else {
                double targetMotorRPS = speed.in(MetersPerSecond) / Swerve.WHEEL_CIRCUMFERENCE * Swerve.DRIVE_GEAR_RATIO;
                double targetMotorFalcon = Conversions.MPSToFalcon(speed.in(MetersPerSecond), Swerve.WHEEL_CIRCUMFERENCE, Swerve.DRIVE_GEAR_RATIO);
                double feedForward = feedforward.calculate(targetMotorRPS);
                driveMotor.set(ControlMode.Velocity, targetMotorFalcon, DemandType.ArbitraryFeedForward, feedForward);
            }
        } else {
            double percentOutput = speed.in(MetersPerSecond) / Swerve.MAX_SPEED;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent jittering
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= SWERVE_IN_PLACE_DRIVE_MPS)
                        ? lastAngle
                        : desiredState.angle;
        angleController.setReference(angle.getDegrees(), CANSparkBase.ControlType.kPosition);

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

        mAngleMotor.setSmartCurrentLimit(Swerve.ANGLE_CONTINUOUS_CURRENT_LIMIT);

        mAngleMotor.setInverted(Swerve.ANGLE_INVERT);
        mAngleMotor.setIdleMode(Swerve.ANGLE_NEUTRAL_MODE);

        integratedAngleEncoder.setPositionConversionFactor(Swerve.ANGLE_CONVERSION_FACTOR);
        angleController.setP(Swerve.ANGLE_KP);
        angleController.setI(Swerve.ANGLE_KI);
        angleController.setD(Swerve.ANGLE_KD);
        angleController.setFF(Swerve.ANGLE_KFF);

        mAngleMotor.enableVoltageCompensation(Swerve.VOLTAGE_COMP);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();

        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        driveMotor.setInverted(Swerve.DRIVE_MOTOR_INVERT);
        driveMotor.setNeutralMode(Swerve.DRIVE_NEUTRAL_MODE);
        driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), Swerve.WHEEL_CIRCUMFERENCE, Swerve.DRIVE_GEAR_RATIO),
                getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Swerve.WHEEL_CIRCUMFERENCE, Swerve.DRIVE_GEAR_RATIO),
                getAngle()
        );
    }
}