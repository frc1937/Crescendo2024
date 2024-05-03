package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.MeasureUtils;

import static edu.wpi.first.units.Units.*;
import static frc.lib.math.Conversions.RPMFromTangentialVelocity;
import static frc.robot.Constants.CanIDConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.FlywheelControlConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final DigitalInput beamBreaker = new DigitalInput(0);
    private final WPI_TalonSRX kickerMotor = new WPI_TalonSRX(KICKER_ID);
    private final Flywheel rightFlywheel = new Flywheel(FLYWHEEL_RIGHT_ID, true, RIGHT_P, RIGHT_S, RIGHT_V, RIGHT_A);
    private final Flywheel leftFlywheel = new Flywheel(FLYWHEEL_LEFT_ID, false, LEFT_P, LEFT_S, LEFT_V, LEFT_A);
    private final Pitch pitch = new Pitch();

    private int consecutiveNoteInsideSamples = 0;

    public ShooterSubsystem() {
        kickerMotor.configFactoryDefault();
        kickerMotor.setNeutralMode(NeutralMode.Brake);
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

        SmartDashboard.putBoolean("shooter/isLoaded", isLoaded());
        SmartDashboard.putBoolean("shooter/areFlywheelsReady", flywheelsAtReference());
        SmartDashboard.putBoolean("shooter/hasPitchArrived", pitchAtReference());
    }

    /**
     * Get the needed pitch theta for the pivot using physics.
     * <p>Formula is taken from
     * <a href="https://en.wikipedia.org/wiki/Projectile_motion?useskin=vector#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">...</a>
     * </p>
     * @param robotPose - The robot's pose, using the correct alliance
     * @param targetPose - the target pose to hit, using the correct alliance
     * @param tangentialVelocity - the tangential velocity of the flywheel
     * @return - the required pitch angle
     */

    public Rotation2d getPitchAnglePhysics(Pose2d robotPose, Pose3d targetPose, double tangentialVelocity) {
        double g = 9.8;
        double vSquared = tangentialVelocity * tangentialVelocity;

        //This is the distance of the pivot off the floor when parallel to the ground
        double z = targetPose.getZ() - Inch.of(8.5).in(Meters);
        double distance = Math.hypot(robotPose.getX() - targetPose.getX(), robotPose.getY() - targetPose.getY());

        double theta = Math.atan(
                (vSquared + Math.sqrt(vSquared*vSquared - g*(g*distance*distance + 2*vSquared*z)))
                / (g*distance)
        );

        return Rotation2d.fromRadians(theta);
    }

    /**
     * We assume the robot isn't moving to get the time of flight
     * @param currentPose - The robot's pose, using the correct alliance
     * @param targetPose - The target pose of the note, using the correct alliance
     * @param tangentialVelocity - The tangential velocity of the flywheels
     * @return - the time of flight in seconds
     */
    public double getTimeOfFlight(Pose2d currentPose, Pose3d targetPose, double tangentialVelocity) {
        Rotation2d theta = getPitchAnglePhysics(currentPose, targetPose, tangentialVelocity);

        double xDiff = targetPose.getX() - currentPose.getX();
        double yDiff = targetPose.getY() - currentPose.getY();

        double distance = Math.hypot(xDiff, yDiff);

        return distance / (tangentialVelocity * Math.cos(theta.getRadians()));
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

    public void setPitchGoal(Rotation2d goal) {
        pitch.setGoal(goal);
    }

    public void setPitchGoal(Rotation2d position, Measure<Velocity<Angle>> velocity) {
        pitch.setGoal(position, velocity);
    }

    public void setReference(Reference reference) {
        pitch.setGoal(reference.pitchPosition, reference.pitchVelocity);
        setTangentialFlywheelsVelocity(reference.flywheelTangentialVelocity);
    }

    public void setPitchConstraints(TrapezoidProfile.Constraints constraints) {
        pitch.setConstraints(constraints);
    }
    
    public void reset() {
        setReference(new Reference());
        stopFlywheels();
        stopKicker();
    }
    
    public Rotation2d getPitchPosition() {
        return pitch.getCurrentPosition();
    }

    public boolean flywheelsAtReference() {
        return leftFlywheel.atSetpoint() && rightFlywheel.atSetpoint();
    }

    public boolean pitchAtReference() {
        return pitch.atGoal();
    }

    public boolean atReference() {
        return pitchAtReference() && flywheelsAtReference();
    }

    public TrapezoidProfile.Constraints getPitchConstraints() {
        return pitch.getConstraints();
    }

    /**
     * Set the target tangential velocity of the flywheels.
     * This method accounts for the difference in wheel size
     * @param tangentialVelocity - the target tangential velocity in metres per second
     */
    public void setTangentialFlywheelsVelocity(Measure<Velocity<Distance>> tangentialVelocity) {
        double leftFlywheelRPM = RPMFromTangentialVelocity(tangentialVelocity.in(MetersPerSecond), LEFT_FLYWHEEL_DIAMETER);
        double rightFlywheelRPM = RPMFromTangentialVelocity(tangentialVelocity.in(MetersPerSecond), RIGHT_FLYWHEEL_DIAMETER);

        leftFlywheel.setSpeed(RPM.of(leftFlywheelRPM));
        rightFlywheel.setSpeed(RPM.of(rightFlywheelRPM));
    }
    
    /**
     * Rotate the flywheels to certain speeds s.t. NOTEs will be released with
     * certain speed and rotation
     *
     * @param speed the average target speed of both flywheels
     * @param spin  the ratio between the right and the left flywheel angular velocities.
     * @param pScalar the scalar of the P part of the PID controller
     */
    public void setFlywheelsSpeed(Measure<Velocity<Angle>> speed, double spin, double pScalar) {
        Measure<Velocity<Angle>> leftSpeed = speed.times(2).divide(spin + 1);
        Measure<Velocity<Angle>> rightSpeed = leftSpeed.times(spin);

        rightFlywheel.setSpeed(rightSpeed, pScalar);
        leftFlywheel.setSpeed(leftSpeed, pScalar);
    }

    public static class Reference implements Interpolatable<Reference> {
        private Rotation2d pitchPosition = PITCH_DEFAULT_ANGLE;
        private Measure<Velocity<Angle>> pitchVelocity = RadiansPerSecond.of(0);
        private Measure<Velocity<Distance>> flywheelTangentialVelocity = MetersPerSecond.of(0);

        public Reference(Rotation2d pitchPosition,
                         Measure<Velocity<Angle>> pitchVelocity,
                         Measure<Velocity<Distance>> flywheelTangentialVelocity) {
            this.pitchPosition = pitchPosition;
            this.pitchVelocity = pitchVelocity;
            this.flywheelTangentialVelocity = flywheelTangentialVelocity;
        }

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
                MeasureUtils.interpolate(pitchVelocity, endValue.pitchVelocity, t),
                MeasureUtils.interpolate(flywheelTangentialVelocity, endValue.flywheelTangentialVelocity, t)
            );
        }
    }
}
