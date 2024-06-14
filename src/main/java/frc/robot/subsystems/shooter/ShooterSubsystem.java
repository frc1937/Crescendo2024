package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.math.Conversions.rpmFromTangentialVelocity;
import static frc.robot.Constants.CanIDConstants.FLYWHEEL_LEFT_ID;
import static frc.robot.Constants.CanIDConstants.FLYWHEEL_RIGHT_ID;
import static frc.robot.Constants.CanIDConstants.KICKER_ID;
import static frc.robot.subsystems.shooter.ShooterConstants.CONSIDERED_NOISELESS_THRESHOLD;
import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_LEFT_P;
import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_LEFT_S;
import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_LEFT_V;
import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_RIGHT_P;
import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_RIGHT_S;
import static frc.robot.subsystems.shooter.ShooterConstants.FLYWHEEL_RIGHT_V;
import static frc.robot.subsystems.shooter.ShooterConstants.LEFT_FLYWHEEL_DIAMETER;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_DEFAULT_ANGLE;
import static frc.robot.subsystems.shooter.ShooterConstants.RIGHT_FLYWHEEL_DIAMETER;

public class ShooterSubsystem extends SubsystemBase {

    private final DigitalInput beamBreaker = new DigitalInput(0);
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(KICKER_ID);

    private final Flywheel rightFlywheel = new Flywheel(FLYWHEEL_RIGHT_ID, true, FLYWHEEL_RIGHT_P, FLYWHEEL_RIGHT_S, FLYWHEEL_RIGHT_V);
    private final Flywheel leftFlywheel = new Flywheel(FLYWHEEL_LEFT_ID, false, FLYWHEEL_LEFT_P, FLYWHEEL_LEFT_S, FLYWHEEL_LEFT_V);
    private final Pitch pitch = new Pitch();

    private int consecutiveNoteInsideSamples = 0;

    public ShooterSubsystem() {
        this.setDefaultCommand(
                startEnd(
                        () -> {
                            resetController();
                            pitch.setGoal(PITCH_DEFAULT_ANGLE);
                            stopFlywheels();
                        },
                        this::resetController
                ));
        configureKickerMotor();
    }

    @Override
    public void periodic() {
        accountForBreakerNoise();

        leftFlywheel.periodic();
        rightFlywheel.periodic();
        pitch.periodic();

        logShooter();
    }

    public void resetController() {
        pitch.resetController();
    }

    public void setPitchBrakeStatus(boolean shouldBrake) {
        pitch.setBrake(shouldBrake);
    }

    public void setPitchVoltage(Measure<Voltage> volts) {
        pitch.setRawVoltage(volts.in(Volts));
    }

    public void setFlywheelVoltage(Measure<Voltage> volts) {
        rightFlywheel.setRawVoltage(volts.in(Volts));
    }

    public void logPitch(SysIdRoutineLog log) {
        log.motor("Pitcher")
                .voltage(pitch.getVoltage())
                .angularPosition(Rotations.of(pitch.getPosition().getRotations()))
                .angularVelocity(RotationsPerSecond.of(pitch.getVelocity()));
    }

    public void logFlywheels(SysIdRoutineLog log) {
        log.motor("FlywheelRight")
                .voltage(rightFlywheel.getVoltage())
                .angularVelocity(rightFlywheel.getVelocity());
    }

    /**
     * @return whether a NOTE is present inside the shooter
     */
    public boolean isLoaded() {
        return consecutiveNoteInsideSamples >= CONSIDERED_NOISELESS_THRESHOLD;
    }

    public void setKickerSpeed(double speed) {
        kickerMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopKicker() {
        kickerMotor.stopMotor();
    }

    public void setFlywheelsSpeed(Measure<Velocity<Angle>> speed) {
        rightFlywheel.setGoal(speed);
        leftFlywheel.setGoal(speed);
    }

    public void stopFlywheels() {
        leftFlywheel.stopMotor();
        rightFlywheel.stopMotor();
    }

    public double getFlywheelsSpeed() {
        return Math.max(rightFlywheel.getVelocity().in(RPM), leftFlywheel.getVelocity().in(RPM));
    }

    public Rotation2d getPitchGoal() {
        return pitch.getGoalPosition();
    }

    public void setReference(Reference reference) {
        pitch.setGoal(reference.pitchPosition);
        setTangentialFlywheelsVelocity(reference.flywheelVelocityMPS);
    }

    public void reset() {
        setReference(new Reference());
        stopFlywheels();
        stopKicker();
    }

    public Pitch getPitch() {
        return pitch;
    }

    public Rotation2d getPitchPosition() {
        return pitch.getPosition();
    }

    public boolean flywheelsAtReference() {
        return leftFlywheel.isAtGoal() && rightFlywheel.isAtGoal();
    }

    public boolean pitchAtReference() {
        return pitch.isAtGoal();
    }

    public boolean atReference() {
        return pitchAtReference() && flywheelsAtReference();
    }

    /**
     * Set the target tangential velocity of the flywheels.
     * This method accounts for the difference in wheel size
     *
     * @param velocityMPS - the target tangential velocity in metres per second
     */
    public void setTangentialFlywheelsVelocity(double velocityMPS) {
        double leftFlywheelRPM = rpmFromTangentialVelocity(velocityMPS, LEFT_FLYWHEEL_DIAMETER);
        double rightFlywheelRPM = rpmFromTangentialVelocity(velocityMPS, RIGHT_FLYWHEEL_DIAMETER);

        leftFlywheel.setGoal(RPM.of(leftFlywheelRPM));
        rightFlywheel.setGoal(RPM.of(rightFlywheelRPM));
    }

    public static class Reference {
        private Rotation2d pitchPosition = PITCH_DEFAULT_ANGLE;
        private double flywheelVelocityMPS = 0;

        public Reference(Rotation2d pitchPosition,
                         double flywheelVelocityMPS) {
            this.pitchPosition = pitchPosition;
            this.flywheelVelocityMPS = flywheelVelocityMPS;
        }

        public Reference(Rotation2d pitchPosition) {
            this.pitchPosition = pitchPosition;
        }

        public Reference() {
        }
    }

    private void configureKickerMotor() {
        kickerMotor.configFactoryDefault();
        kickerMotor.setNeutralMode(NeutralMode.Brake);
    }

    private void logShooter() {
        SmartDashboard.putBoolean("shooter/isLoaded", isLoaded());
        SmartDashboard.putBoolean("shooter/areFlywheelsReady", flywheelsAtReference());
        SmartDashboard.putBoolean("shooter/hasPitchArrived", pitchAtReference());
    }

    private void accountForBreakerNoise() {
        if (!beamBreaker.get()) {
            consecutiveNoteInsideSamples++;
            return;
        }

        consecutiveNoteInsideSamples = 0;
    }
}
