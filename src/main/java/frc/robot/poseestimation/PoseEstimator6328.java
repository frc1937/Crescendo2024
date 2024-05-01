package frc.robot.poseestimation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.NoSuchElementException;
import java.util.Optional;

import static frc.robot.Constants.VisionConstants.STATES_AMBIGUITY;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_KINEMATICS;

public class PoseEstimator6328 {
    public record OdometryObservation(
            SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {
    }

    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    }

    private static final double POSE_BUFFER_SIZE_SECONDS = 2.0;

    // Pose Estimation Members
    private Pose2d odometryPose = new Pose2d();
    private Pose2d estimatedPose = new Pose2d();

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SECONDS);

    //Kalman Filter
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

    private SwerveDriveWheelPositions lastWheelPositions =
            new SwerveDriveWheelPositions(
                    new SwerveModulePosition[]{
                            new SwerveModulePosition(),
                            new SwerveModulePosition(),
                            new SwerveModulePosition(),
                            new SwerveModulePosition()
                    }
                    );

    private Rotation2d lastGyroAngle = new Rotation2d();

    public PoseEstimator6328() {
        for (int i = 0; i < 3; ++i) {
            qStdDevs.set(i, 0, Math.pow(STATES_AMBIGUITY.get(i, 0), 2));
        }
    }

    /**
     * Add odometry observation
     */
    public void addOdometryObservation(OdometryObservation observation) {
        Pose2d lastOdometryPose = odometryPose;
        Twist2d twist = SWERVE_KINEMATICS.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        // Check gyro connected
        if (observation.gyroAngle != null) {
            // Update dtheta for twist if gyro connected
            twist =
                    new Twist2d(
                            twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
            lastGyroAngle = observation.gyroAngle();
        }
        // Add twist to odometry pose
        odometryPose = odometryPose.exp(twist);
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);
        // Calculate diff from last odometry pose and add onto pose estimate
        estimatedPose = estimatedPose.exp(lastOdometryPose.log(odometryPose));
    }

    public void addVisionObservation(VisionObservation observation) {
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - POSE_BUFFER_SIZE_SECONDS > observation.timestamp()) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }
        // Get odometry based pose at timestamp
        Optional<Pose2d> sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty())
            // exit if not there
            return;

        // sample --> odometryPose transform and backwards of that
        Transform2d sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        Transform2d odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        // get old estimate by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Calculate 3 x 3 vision matrix
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
        }
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }
        // difference between estimate and vision pose
        Twist2d twist = estimateAtTime.log(observation.visionPose());
        // scale twist by visionK
        var kTimesTwist = visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));
        Twist2d scaledTwist =
                new Twist2d(kTimesTwist.get(0, 0), kTimesTwist.get(1, 0), kTimesTwist.get(2, 0));

        // Recalculate current estimate by applying scaled twist to old estimate
        // then replaying odometry data
        estimatedPose = estimateAtTime.exp(scaledTwist).plus(sampleToOdometryTransform);
    }

    /**
     * Reset estimated pose and odometry pose to pose <br>
     * Clear pose buffer
     */
    public void resetPose(Pose2d initialPose) {
        estimatedPose = initialPose;
        odometryPose = initialPose;
        poseBuffer.clear();
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public Pose2d getOdometryPose() {
        return odometryPose;
    }
}

/*
Hi. For my FRC project, I do constants in the following way.
I create a constants class in a folder alongside the matching subsystem.
However, sometimes unrelated classes require the use of the subsystem constants, which are package protected.
How should I solve this?
I can either:
1. Make one big constants class for all subsystems (UGLY)
2. Keep the current architecture but make a general-constants class with all of the cross-constants. (UGLY TOO)
- Sometihng else
Please help! thanks.
 */