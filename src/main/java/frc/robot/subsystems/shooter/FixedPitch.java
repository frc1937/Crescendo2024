package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Volts;
import static frc.lib.util.CTREUtil.applyConfig;
import static frc.robot.Constants.CanIDConstants.PIVOT_CAN_CODER;
import static frc.robot.Constants.CanIDConstants.PIVOT_ID;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KA;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KG;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KS;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_KV;
import static frc.robot.subsystems.shooter.ShooterConstants.PIVOT_ENCODER_OFFSET;

public class FixedPitch {
    private static double TIME_DIFFERENCE = 0.02;

    private final CANSparkFlex motor = new CANSparkFlex(PIVOT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANcoder absoluteEncoder = new CANcoder(PIVOT_CAN_CODER);

    private final ArmFeedforward feedforward = new ArmFeedforward(PITCH_KS, PITCH_KG, PITCH_KV, PITCH_KA);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10, 20);

    private final ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0,
            constraints, TIME_DIFFERENCE
    );

    private StatusSignal<Double> encoderPositionSignal, encoderVelocitySignal;

    public FixedPitch() {
        configurePitchMotor();
        configureSteerEncoder();
        configureSoftLimits();
    }

    public void periodic() {
        double controllerOutput = controller.calculate(getPosition().getRotations());
        double feedforwardOutput = feedforward.calculate(
                controller.getSetpoint().position,
                controller.getSetpoint().velocity
        );

        SmartDashboard.putNumber("pitch/controllerOutput", controllerOutput);
        SmartDashboard.putNumber("pitch/feedforwardOutput", feedforwardOutput);

        motor.setVoltage(controllerOutput + feedforwardOutput);
    }

    public boolean isAtGoal() {
        return controller.atGoal();
    }

    public void setGoal(Rotation2d position) {
        setGoal(position, 0);
    }

    public void setGoal(Rotation2d position, double velocity) {
        controller.setGoal(new TrapezoidProfile.State(position.getRotations(), velocity));
    }

    public Rotation2d getGoalPosition() {
        return Rotation2d.fromRotations(controller.getGoal().position);
    }

    public Rotation2d getPosition() {
        Rotation2d angle = Rotation2d.fromRotations(encoderPositionSignal.refresh().getValue()).minus(PIVOT_ENCODER_OFFSET);

        //Ensure the encoder always has the same starting angle.
        if (angle.getDegrees() > 300)
            angle.minus(Rotation2d.fromDegrees(360));

        return angle;
    }

    public double getVelocity() {
        return encoderVelocitySignal.refresh().getValue();
    }

    public Measure<Voltage> getVoltage() {
        return Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    }

    public void drivePitch(double voltage) {
        motor.setVoltage(voltage);
    }

    private void configureSteerEncoder() {
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        applyConfig(absoluteEncoder, canCoderConfig);

        encoderPositionSignal = absoluteEncoder.getPosition().clone();
        encoderVelocitySignal = absoluteEncoder.getVelocity().clone();

        encoderPositionSignal.setUpdateFrequency(50);
        encoderVelocitySignal.setUpdateFrequency(50);

        absoluteEncoder.optimizeBusUtilization();
    }

    private void configurePitchMotor() {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.setSmartCurrentLimit(40);
    }

    private void configureSoftLimits() {
        motor.getEncoder().setPosition(getPosition().getRotations());
    }
}
