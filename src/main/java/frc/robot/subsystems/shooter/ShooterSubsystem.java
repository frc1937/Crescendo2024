package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.MeasureUtils;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.lib.math.Conversions.rpmFromTangentialVelocity;
import static frc.robot.Constants.CanIDConstants.FLYWHEEL_LEFT_ID;
import static frc.robot.Constants.CanIDConstants.FLYWHEEL_RIGHT_ID;
import static frc.robot.Constants.CanIDConstants.KICKER_ID;
import static frc.robot.subsystems.shooter.ShooterConstants.CONSIDERED_NOISELESS_THRESHOLD;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelControlConstants.LEFT_A;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelControlConstants.LEFT_P;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelControlConstants.LEFT_S;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelControlConstants.LEFT_V;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelControlConstants.RIGHT_A;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelControlConstants.RIGHT_P;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelControlConstants.RIGHT_S;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelControlConstants.RIGHT_V;
import static frc.robot.subsystems.shooter.ShooterConstants.LEFT_FLYWHEEL_DIAMETER;
import static frc.robot.subsystems.shooter.ShooterConstants.PITCH_DEFAULT_ANGLE;
import static frc.robot.subsystems.shooter.ShooterConstants.RIGHT_FLYWHEEL_DIAMETER;

public class ShooterSubsystem extends SubsystemBase {
    private final DigitalInput beamBreaker = new DigitalInput(0);
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(KICKER_ID);
    private final Flywheel rightFlywheel = new Flywheel(FLYWHEEL_RIGHT_ID, true, RIGHT_P, RIGHT_S, RIGHT_V, RIGHT_A);
    private final Flywheel leftFlywheel = new Flywheel(FLYWHEEL_LEFT_ID, false, LEFT_P, LEFT_S, LEFT_V, LEFT_A);
    private final Pitch pitch = new Pitch();

    private int consecutiveNoteInsideSamples = 0;

    public ShooterSubsystem() {
        configureKickerMotor();
    }

    @Override
    public void periodic() {
        if (!beamBreaker.get()) {
            consecutiveNoteInsideSamples++;
        } else {
            consecutiveNoteInsideSamples = 0;
        }

        leftFlywheel.periodic();
        rightFlywheel.periodic();
        pitch.periodic();

        logShooter();
    }

    public void setPitchVoltage(Measure<Voltage> volts) {
        pitch.drivePitch(volts.in(Volts));
    }

    public void logPitch(SysIdRoutineLog log) {
        log.motor("Pitcher")
                .voltage(pitch.getVoltage())
                .angularPosition(Rotations.of(pitch.getPosition().getRotations()))
                .angularVelocity(RotationsPerSecond.of(pitch.getVelocity()));
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
        rightFlywheel.setSpeed(speed);
        leftFlywheel.setSpeed(speed);
    }

    public void stopFlywheels() {
        leftFlywheel.stopMotor();
        rightFlywheel.stopMotor();
    }

    public double getFlywheelsSpeed() {
        return Math.max(rightFlywheel.getSpeed().in(RPM), leftFlywheel.getSpeed().in(RPM));
    }

    public Rotation2d getPitchGoal() {
        return pitch.getGoalPosition();
    }

    public void setPitchGoal(Rotation2d goal) {
        pitch.setGoal(goal);
    }

    public void setReference(Reference reference) {
        pitch.setGoal(reference.pitchPosition);
        setTangentialFlywheelsVelocity(reference.flywheelTangentialVelocity);
    }

    public void reset() {
        setReference(new Reference());
        stopFlywheels();
        stopKicker();
    }
    
    public Rotation2d getPitchPosition() {
        return pitch.getPosition();
    }

    public boolean flywheelsAtReference() {
        return leftFlywheel.atSetpoint() && rightFlywheel.atSetpoint();
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
     * @param tangentialVelocity - the target tangential velocity in metres per second
     */
    public void setTangentialFlywheelsVelocity(Measure<Velocity<Distance>> tangentialVelocity) {
        double leftFlywheelRPM = rpmFromTangentialVelocity(tangentialVelocity, LEFT_FLYWHEEL_DIAMETER);
        double rightFlywheelRPM = rpmFromTangentialVelocity(tangentialVelocity, RIGHT_FLYWHEEL_DIAMETER);

        leftFlywheel.setSpeed(RPM.of(leftFlywheelRPM));
        rightFlywheel.setSpeed(RPM.of(rightFlywheelRPM));
    }

    public static class Reference implements Interpolatable<Reference> {
        private Rotation2d pitchPosition = PITCH_DEFAULT_ANGLE;
        private Measure<Velocity<Distance>> flywheelTangentialVelocity = MetersPerSecond.of(0);

        public Reference(Rotation2d pitchPosition,
                         Measure<Velocity<Distance>> flywheelTangentialVelocity) {
            this.pitchPosition = pitchPosition;
            this.flywheelTangentialVelocity = flywheelTangentialVelocity;
        }
                
        public Reference(Rotation2d pitchPosition) {
            this.pitchPosition = pitchPosition;
        }

        public Reference() {}

        @Override
        public Reference interpolate(Reference endValue, double t) {
            return new Reference(
                pitchPosition.interpolate(endValue.pitchPosition, t),
                MeasureUtils.interpolate(flywheelTangentialVelocity, endValue.flywheelTangentialVelocity, t)
            );
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
}
