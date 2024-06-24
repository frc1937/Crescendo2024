package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.generic.Properties;
import frc.lib.generic.encoder.Encoder;
import frc.lib.generic.encoder.EncoderConfiguration;
import frc.lib.generic.encoder.EncoderProperties;
import frc.lib.generic.encoder.GenericCanCoder;
import frc.lib.generic.motor.GenericSpark;
import frc.lib.generic.motor.GenericTalonFX;
import frc.lib.generic.motor.Motor;
import frc.lib.generic.motor.MotorConfiguration;
import frc.lib.generic.motor.MotorProperties;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;

import static frc.lib.math.Conversions.metersPerSecondToRotationsPerSecond;
import static frc.robot.GlobalConstants.ODOMETRY_FREQUENCY_HERTZ;
import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_CURRENT_LIMIT;
import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_GEAR_RATIO;
import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_KP;
import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_MOTOR_INVERT;
import static frc.robot.subsystems.swerve.SwerveConstants.ANGLE_NEUTRAL_MODE;
import static frc.robot.subsystems.swerve.SwerveConstants.CAN_CODER_INVERT;
import static frc.robot.subsystems.swerve.SwerveConstants.CLOSED_LOOP_RAMP;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_GEAR_RATIO;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KA;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KD;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KI;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KP;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KS;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_KV;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_MOTOR_INVERT;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_NEUTRAL_MODE;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_STATOR_CURRENT_LIMIT;
import static frc.robot.subsystems.swerve.SwerveConstants.DRIVE_SUPPLY_CURRENT_LIMIT;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve.SwerveConstants.OPEN_LOOP_RAMP;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_IN_PLACE_DRIVE_MPS;
import static frc.robot.subsystems.swerve.SwerveConstants.WHEEL_CIRCUMFERENCE;

public class SwerveModule5990 {
    private static final double WHEEL_DIAMETER = WHEEL_CIRCUMFERENCE / Math.PI;

    private int highTemperatureCounter;

    public final SwerveConstants.SwerveModuleConstants swerveModuleConstants;

    private Motor driveMotor;
    private Motor steerMotor;

    private Encoder steerAbsoluteEncoder;

    private SwerveModuleState targetState;

    public SwerveModule5990(SwerveConstants.SwerveModuleConstants constants) {
        this.swerveModuleConstants = constants;

        configureSteerAbsoluteEncoder();
        configureDriveMotor();
        configureSteerMotor();
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
        return Rotation2d.fromRotations(steerAbsoluteEncoder.getEncoderPosition());
    }

    public void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    public void periodic() {
        this.keepTemperatureInCheck();

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
        steerMotor = new GenericSpark(swerveModuleConstants.steerMotorID());

        MotorConfiguration steerConfiguration = new MotorConfiguration();

        steerConfiguration.supplyCurrentLimit = ANGLE_CURRENT_LIMIT;
        steerConfiguration.inverted = ANGLE_MOTOR_INVERT;
        steerConfiguration.idleMode = ANGLE_NEUTRAL_MODE;

        steerConfiguration.conversionFactor = ANGLE_GEAR_RATIO;

        steerMotor.setMotorPosition(getCurrentAngle().getRotations());
        steerMotor.setSignalUpdateFrequency(Properties.SignalType.POSITION, 50);

        steerMotor.configure(steerConfiguration);
    }

    private void configureSteerAbsoluteEncoder() {
        steerAbsoluteEncoder = new GenericCanCoder(swerveModuleConstants.canCoderID());

        EncoderConfiguration encoderConfiguration = new EncoderConfiguration();

        encoderConfiguration.invert = CAN_CODER_INVERT;
        encoderConfiguration.sensorRange = EncoderProperties.SensorRange.ZeroToOne;
        encoderConfiguration.offsetRotations = -swerveModuleConstants.angleOffset();

        steerAbsoluteEncoder.configure(encoderConfiguration);

        steerAbsoluteEncoder.setSignalUpdateFrequency(Properties.SignalType.POSITION, ODOMETRY_FREQUENCY_HERTZ);
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

        driveMotor.setSignalUpdateFrequency(Properties.SignalType.VELOCITY, ODOMETRY_FREQUENCY_HERTZ);
        driveMotor.setSignalUpdateFrequency(Properties.SignalType.POSITION, ODOMETRY_FREQUENCY_HERTZ);
        driveMotor.setSignalUpdateFrequency(Properties.SignalType.TEMPERATURE, 50);

        driveMotor.configure(driveConfiguration);
    }

    public void keepTemperatureInCheck() {
        double currentTemperature = (float) driveMotor.getTemperature();

        if (currentTemperature > 70) highTemperatureCounter++;
        else highTemperatureCounter = 0;

        if (highTemperatureCounter > 200) {
            System.out.println("Module #" + swerveModuleConstants.moduleNumber() + " is at " + currentTemperature + " Celsius");
        }
    }
}
