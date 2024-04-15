package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule5990;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.poseestimation.TalonFXOdometryThread6328;
import frc.robot.util.AllianceUtilities;

import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.Constants.DRIVE_NEUTRAL_DEADBAND;
import static frc.robot.Constants.ROTATION_NEUTRAL_DEADBAND;
import static frc.robot.Constants.Swerve.AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG;
import static frc.robot.Constants.Swerve.SWERVE_KINEMATICS;

public class Swerve5990 extends SubsystemBase {
    private final Lock odometryLock = new ReentrantLock();
    private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.Swerve.PIGEON_ID);
    private final Queue<Double> timestampQueue = TalonFXOdometryThread6328.getInstance(odometryLock).getTimestampQueue();
    private final PoseEstimator poseEstimator;

    private SwerveModule5990[] modules = new SwerveModule5990[3];

    public Swerve5990(PoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;

        gyro.setYaw(AllianceUtilities.AlliancePose2d.fromBlueAlliancePose(new Pose2d(0, 0, new Rotation2d())).toAlliancePose().getRotation().getDegrees());

        configurePathPlanner();
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        updateModules();
        odometryLock.unlock();

        updatePoseEstimator();
    }

    public void driveSelfRelative(ChassisSpeeds speeds) {
        ChassisSpeeds discretizedChassisSpeeds = discretise(speeds);

        SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(discretizedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule5990 mod : modules) {
            mod.setTargetState(swerveModuleStates[mod.swerveModuleConstants.moduleNumber], true);
        }
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }

    public void lockSwerve() {
        final SwerveModuleState
                right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

        modules[0].setTargetState(left, false);
        modules[1].setTargetState(right, false);
        modules[2].setTargetState(right, false);
        modules[3].setTargetState(left, false);
    }

    public Rotation2d getYaw() {
        return gyro.getRotation2d();
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> poseEstimator.getCurrentPose().toBlueAlliancePose(),
                pose -> poseEstimator.resetPose(AllianceUtilities.AlliancePose2d.fromAlliancePose(pose)), //TODO LOL THIS MIGHT BE RLY WRONG

                this::getSelfRelativeVelocity,
                this::driveSelfRelative,

                HOLONOMIC_PATH_FOLLOWER_CONFIG,
                () -> !AllianceUtilities.isBlueAlliance(),
                this
        );
    }

    private void updateModules() {
        for(SwerveModule5990 module : modules) {
            module.periodic();
        }
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (SwerveModule5990 mod : modules) {
            states[mod.swerveModuleConstants.moduleNumber] = mod.getCurrentState(); //TODO: Make a getState func
        }

        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (SwerveModule5990 mod : modules) {
            positions[mod.swerveModuleConstants.moduleNumber] = mod.getCurrentPosition();
        }

        return positions;
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds the chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    private ChassisSpeeds discretise(ChassisSpeeds chassisSpeeds) {
        return ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    }

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) <= DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= ROTATION_NEUTRAL_DEADBAND;
    }

    private void updatePoseEstimator() {
        final int odometryUpdates = swerveInputs.odometryUpdatesYawDegrees.length;
        final SwerveDriveWheelPositions[] swerveWheelPositions = new SwerveDriveWheelPositions[odometryUpdates];
        final Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = getSwerveWheelPositions(i);
            gyroRotations[i] = Rotation2d.fromDegrees(swerveInputs.odometryUpdatesYawDegrees[i]);
        }

        poseEstimator.updateFromOdometry(swerveWheelPositions, gyroRotations, swerveInputs.odometryUpdatesTimestamp);
    }

    private SwerveDriveWheelPositions getSwerveWheelPositions(int odometryUpdateIndex) {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modules.length];

        for(SwerveModule5990 module : modules) {
            swerveModulePositions[module.swerveModuleConstants.moduleNumber] = module.getOdometryPosition(odometryUpdateIndex);
        }

        return new SwerveDriveWheelPositions(swerveModulePositions);
    } //todo: GetOdo pose method. odoUpdate Yaw method. odoUpdateTimestamp method.
}
