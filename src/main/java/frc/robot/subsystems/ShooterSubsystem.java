package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Flywheel;
import frc.robot.Pitch;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.CONSIDERED_NOISELESS_THRESHOLD;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_LEFT_ID;
import static frc.robot.Constants.ShootingConstants.FLYWHEEL_RIGHT_ID;
import static frc.robot.Constants.ShootingConstants.FlywheelControlConstants.LEFT_A;
import static frc.robot.Constants.ShootingConstants.FlywheelControlConstants.LEFT_P;
import static frc.robot.Constants.ShootingConstants.FlywheelControlConstants.LEFT_S;
import static frc.robot.Constants.ShootingConstants.FlywheelControlConstants.LEFT_V;
import static frc.robot.Constants.ShootingConstants.FlywheelControlConstants.RIGHT_A;
import static frc.robot.Constants.ShootingConstants.FlywheelControlConstants.RIGHT_P;
import static frc.robot.Constants.ShootingConstants.FlywheelControlConstants.RIGHT_S;
import static frc.robot.Constants.ShootingConstants.FlywheelControlConstants.RIGHT_V;
import static frc.robot.Constants.ShootingConstants.KICKER_ID;
import static frc.robot.Constants.ShootingConstants.PITCH_DEFAULT_ANGLE;

public class ShooterSubsystem extends SubsystemBase {
    private final DigitalInput beamBreaker = new DigitalInput(0);
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(KICKER_ID);
    private final Flywheel rightFlywheel = new Flywheel(FLYWHEEL_RIGHT_ID, false, RIGHT_P, RIGHT_S, RIGHT_V, RIGHT_A);
    private final Flywheel leftFlywheel = new Flywheel(FLYWHEEL_LEFT_ID, true, LEFT_P, LEFT_S, LEFT_V, LEFT_A);
    private final Pitch pitch = new Pitch();
    private int consecutiveNoteInsideSamples = 0;
    private final LedSubsystem ledSubsystem = new LedSubsystem();

    public final Measure<Velocity<Angle>> theoreticalMaximumVelocity =
        rightFlywheel.theoreticalMaximumVelocity.unit().of(
            Math.min(rightFlywheel.theoreticalMaximumVelocity.baseUnitMagnitude(),
                     leftFlywheel.theoreticalMaximumVelocity.baseUnitMagnitude()));
    
    public ShooterSubsystem() {
        kickerMotor.configFactoryDefault();
        kickerMotor.setNeutralMode(NeutralMode.Brake);


        // pitch.setPosition(Rotation2d.fromDegrees(45));
    }

    @Override
    public void periodic() {
        if (isLoaded()) {
            ledSubsystem.setLedColour(255, 80, 0);
        } else {
            ledSubsystem.setLedColour(0, 0, 255);
        }

        if (!beamBreaker.get()) {
            consecutiveNoteInsideSamples++;
        } else {
            consecutiveNoteInsideSamples = 0;
        };

        leftFlywheel.periodic();
        rightFlywheel.periodic();
        pitch.periodic();
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

    /**
     * Rotate the flywheels to certain speeds s.t. NOTEs will be released with
     * certain speed and rotation
     *
     * @param speed the average target speed of both flywheels
     * @param differencePercents  a value in range [0, 1] where (1 - differencePercents) = (right speed / left speed).
     *              Thus, the difference between the left and right speeds is proprtional to {@code speed}.
     */
    public void setFlywheelsSpeed(Measure<Velocity<Angle>> speed, double differencePercents) {
        Measure<Velocity<Angle>> leftSpeed = speed.times(2).divide(2.d - differencePercents);
        Measure<Velocity<Angle>> rightSpeed = leftSpeed.times(1.d - differencePercents);

        rightFlywheel.setSpeed(rightSpeed);
        leftFlywheel.setSpeed(leftSpeed);
    }

    public void stopFlywheels() {
        leftFlywheel.stopMotor();
        rightFlywheel.stopMotor();
    }

    public boolean areFlywheelsReady() {
        return leftFlywheel.atSetpoint() && rightFlywheel.atSetpoint();
    }

    public boolean isPitchReady() {
        return pitch.atGoal();
    }

    public void setPitchGoal(Rotation2d goal) {
        // SmartDashboard.putNumber("pitch/Goal", goal.getDegrees());
        pitch.setGoal(goal);
    }

    public Rotation2d getPitchPosition() {
        return pitch.getCurrentPosition();
    }

    public void setPitchGoal(Measure<Angle> position, Measure<Velocity<Angle>> velocity) {
        pitch.setGoal(position, velocity);
    }

    public void reset() {
        stopFlywheels();
        stopKicker();
        setPitchGoal(Rotation2d.fromDegrees(PITCH_DEFAULT_ANGLE));
    }
}
