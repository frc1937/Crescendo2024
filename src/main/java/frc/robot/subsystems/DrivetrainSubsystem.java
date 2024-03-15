package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.vision.VisionPoseEstimator;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

import static frc.robot.Constants.ShootingConstants.POSE_HISTORY_DURATION;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.VisionConstants.FRONT_CAMERA_NAME;
import static frc.robot.Constants.VisionConstants.REAR_CAMERA_NAME;

public class DrivetrainSubsystem extends SubsystemBase {
    public final SwerveDrivePoseEstimator poseEstimator;
    public final SwerveModule[] swerveModules;
    public final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.Swerve.PIGEON_ID);
    public final VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator();
    private final Field2d field2d = new Field2d();
    private final TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(POSE_HISTORY_DURATION);
    private double previousTimestamp = 0;
    private final ProfiledPIDController azimuthController = new ProfiledPIDController(
            AZIMUTH_CONTROLLER_P, AZIMUTH_CONTROLLER_I, AZIMUTH_CONTROLLER_D,
            AZIMUTH_CONTROLLER_CONSTRAINTS);

    private double yawCorrection = 0;

    public DrivetrainSubsystem() {
        SmartDashboard.putData("Field", field2d);

        gyro.configFactoryDefault();
        zeroGyro();

        swerveModules = new SwerveModule[]{
                new SwerveModule(0, Constants.Swerve.Module0.CONSTANTS),
                new SwerveModule(1, Constants.Swerve.Module1.CONSTANTS),
                new SwerveModule(2, Constants.Swerve.Module2.CONSTANTS),
                new SwerveModule(3, Constants.Swerve.Module3.CONSTANTS)
        };

        Timer.delay(1.0);
        resetModulesToAbsolute();

        poseEstimator = new SwerveDrivePoseEstimator(
                SWERVE_KINEMATICS,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                Constants.VisionConstants.STATE_STANDARD_DEVIATIONS,
                Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS
        );

        AutoBuilder.configureHolonomic(
                this::getPose,
                pose -> poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose),
                () -> SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates()),
                this::drive,
                AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                () -> {
                    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
                    return alliance == DriverStation.Alliance.Red;
                },
                this
        );

        azimuthController.setTolerance(AZIMUTH_CONTROLLER_TOLERANCE);
        azimuthController.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putNumber("Azimuth Error [rad]", azimuthController.getPositionError());
    }

    /**
     * Publish the current PID gains so they can be modified
     */
    public void publishControllerGains() {
        SmartDashboard.putNumber("swerve/azimuth-controller/p", AZIMUTH_CONTROLLER_P);
        SmartDashboard.putNumber("swerve/azimuth-controller/i", AZIMUTH_CONTROLLER_I);
        SmartDashboard.putNumber("swerve/azimuth-controller/d", AZIMUTH_CONTROLLER_D);
    }

    /**
     * Re-read the PID gains without re-deploying the code.
     */
    public void rereadControllerGains() {
        azimuthController.setPID(
                SmartDashboard.getNumber("swerve/azimuth-controller/p", AZIMUTH_CONTROLLER_P),
                SmartDashboard.getNumber("swerve/azimuth-controller/i", AZIMUTH_CONTROLLER_I),
                SmartDashboard.getNumber("swerve/azimuth-controller/d", AZIMUTH_CONTROLLER_D));
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean closedLoop) {
        ChassisSpeeds discretizedChassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        
        SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(discretizedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], closedLoop);
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        drive(chassisSpeeds, false); //todo: CHECK WHICH VALUE's CORRECT, CHANGED THIS TO FALSE
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean closedLoop) {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red)
            translation = translation.unaryMinus();

        if (fieldRelative) {
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getGyroYaw());
            drive(chassisSpeeds, closedLoop);
        } else {
            drive(new ChassisSpeeds(translation.getX(), translation.getY(), rotation), closedLoop);
        }
    }

    /**
     * Set the azimuth goal.
     *
     * @param azimuthGoal field-relative goal azimuth, not flipped by alliance
     * @see #driveWithAzimuth(Translation2d)
     */
    public void setAzimuthGoal(Rotation2d azimuthGoal) {
        SmartDashboard.putNumber("Azimuth Goal [deg]", azimuthGoal.getDegrees());
        azimuthController.setGoal(azimuthGoal.getRadians());
    }

    /**
     * Drive the robot whilst rotating it to achieve the goal azimuth
     *
     * @param translation field-relative translation, like in {@link #drive(Translation2d, double, boolean, boolean) drive}
     * @see #setAzimuthGoal(Rotation2d)
     */
    public void driveWithAzimuth(Translation2d translation) {
        drive(translation, yawCorrection, true, true);
    }

    /**
     * Drive the robot whilst rotating it to achieve the goal azimuth
     *
     * @param translation field-relative translation, like in {@link #drive(Translation2d, double, boolean, boolean) drive}
     * @param azimuthGoal field-relative goal azimuth, not flipped by alliance
     * @see #setAzimuthGoal(Rotation2d)
     * @see #driveWithAzimuth(Translation2d)
     */
    public void driveWithAzimuth(Translation2d translation, Rotation2d azimuthGoal) {
        setAzimuthGoal(azimuthGoal);
        driveWithAzimuth(translation);
    }

    public boolean azimuthAtGoal() {
        return Math.abs(azimuthController.getPositionError()) < AZIMUTH_CONTROLLER_TOLERANCE;
    }

    public void resetPose() {
        poseEstimator.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(), new Pose2d());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }

        return positions;
    }

    public void zeroGyro() {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            gyro.setYaw(0);
        } else {
            gyro.setYaw(180);
        }

    }

    // TODO rename to getGyroAzimuth
    public Rotation2d getGyroYaw() {
        return Constants.Swerve.INVERT_GYRO ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro", gyro.getYaw());

        Optional<EstimatedRobotPose> estimatedFrontPose = visionPoseEstimator.estimateGlobalPose(poseEstimator.getEstimatedPosition(), FRONT_CAMERA_NAME);
        estimatedFrontPose.ifPresent(this::updateByEstimator);

        Optional<EstimatedRobotPose> estimatedRearPose = visionPoseEstimator.estimateGlobalPose(poseEstimator.getEstimatedPosition(), REAR_CAMERA_NAME);
        estimatedRearPose.ifPresent(this::updateByEstimator);

        poseEstimator.update(getGyroYaw(), getModulePositions());

        field2d.setRobotPose(getPose());

        SmartDashboard.putData("Field", field2d);

        // Calculate the azimuth control. Whilst it is always calculated, only
        // {@link #driveWithAzimuth driveWithAzimuth} uses it.
        SmartDashboard.putNumber("swerve/azimuth [deg]", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("swerve/azimuth (gyro) [deg]", MathUtil.inputModulus(getGyroYaw().getDegrees(), -180, 180));
        yawCorrection = azimuthController.calculate(getPose().getRotation().getRadians());
    }

    public TimeInterpolatableBuffer<Pose2d> getPoseHistory() {
        return poseHistory;
    }

    public void infrequentPeriodic() {
        sampleRobotPose();
    }

    public void stop() {
        drive(new ChassisSpeeds(0, 0, 0), false);
    }

    private void updateByEstimator(EstimatedRobotPose estimatedRobotPose) {
        if (estimatedRobotPose == null) return;

        double currentTimestamp = estimatedRobotPose.timestampSeconds;

        if (previousTimestamp != currentTimestamp) {
            previousTimestamp = currentTimestamp;

            Pose3d visionPose = estimatedRobotPose.estimatedPose;
            poseEstimator.addVisionMeasurement(visionPose.toPose2d(),
                    Timer.getFPGATimestamp(),
                    visionPoseEstimator.confidenceCalculator(estimatedRobotPose)
            );
        }
    }

    private void sampleRobotPose() {
        poseHistory.addSample(Timer.getFPGATimestamp(), getPose());
    }
}
