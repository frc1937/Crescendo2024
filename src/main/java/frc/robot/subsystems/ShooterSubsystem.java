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
import static frc.robot.Constants.ShootingConstants.PIVOT_ID;

public class ShooterSubsystem extends SubsystemBase {
    private final DigitalInput beamBreaker = new DigitalInput(0);
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(KICKER_ID);
    private final Flywheel rightFlywheel = new Flywheel(FLYWHEEL_RIGHT_ID, false, RIGHT_P, RIGHT_S, RIGHT_V, RIGHT_A);
    private final Flywheel leftFlywheel = new Flywheel(FLYWHEEL_LEFT_ID, true, LEFT_P, LEFT_S, LEFT_V, LEFT_A);
    private final Pitch pitch = new Pitch(PIVOT_ID, false, 1, 0.05, 0.05, 0.05);
    private int consecutiveNoteInsideSamples = 0;
    private final LedSubsystem ledSubsystem = new LedSubsystem();

    public ShooterSubsystem() {
        configureSRXMotor(kickerMotor);
    }

    @Override
    public void periodic() {
        if (doesSeeNoteNoiseless()) {
            ledSubsystem.setLedColour(255, 80, 0);
        } else {
            ledSubsystem.setLedColour(0, 0, 255);
        }

        /* FOR DEBUGGING, REMOVE */
        SmartDashboard.putNumber("Current Angle Pitch", pitch.getPosition().getDegrees());
        SmartDashboard.putNumber("Setpoint Pitch", pitch.getSetpoint().getDegrees());
        SmartDashboard.putNumber("left flywheel rpm", leftFlywheel.getSpeed().in(RPM));
        SmartDashboard.putBoolean("Does see note", doesSeeNote());

        if (doesSeeNote()) consecutiveNoteInsideSamples++;
        else consecutiveNoteInsideSamples = 0;

        leftFlywheel.periodic();
        rightFlywheel.periodic();

        pitch.periodic();
    }

    public boolean doesSeeNoteNoiseless() {
        return consecutiveNoteInsideSamples >= CONSIDERED_NOISELESS_THRESHOLD;
    }

    public void setKickerSpeed(double speed) {
        kickerMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopKicker() {
        kickerMotor.stopMotor();
    }

    public void setFlywheelsSpeed(Measure<Velocity<Angle>> speed) {
//        rightFlywheel.setSpeed(speed);
//        leftFlywheel.setSpeed(speed);
    }

    /**
     * Rotate the flywheels to certain speeds s.t. NOTEs will be released with
     * certain speed and rotation
     *
     * @param speed the average target speed of both flywheels
     * @param spin  a value in range [0, 1] where (1 - spin) = (right speed / left speed). Thus,
     *              the difference between the left and right speeds is proprtional to
     *              {@code speed}.
     */
    public void setFlywheelsSpeed(Measure<Velocity<Angle>> speed, double spin) {
//        Measure<Velocity<Angle>> leftSpeed = speed.times(2).divide(2.d - spin);
//        Measure<Velocity<Angle>> rightSpeed = leftSpeed.times(1.d - spin);
//
//        rightFlywheel.setSpeed(rightSpeed);
//        leftFlywheel.setSpeed(leftSpeed);
    }

    public void stopFlywheels() {
//        leftFlywheel.stopMotor();
//        rightFlywheel.stopMotor();
    }

    public boolean areFlywheelsReady() {
        return true;
//        return leftFlywheel.atSetpoint() && rightFlywheel.atSetpoint();
    }

    public boolean hasPivotArrived() {
        return pitch.atSetpoint();
    }

    public void setPitchPosition(Rotation2d rotation2d) {
        pitch.setPosition(rotation2d);
    }

    private void configureSRXMotor(WPI_TalonSRX motor) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
    }

    private boolean doesSeeNote() {
        return !beamBreaker.get();
    }
}
