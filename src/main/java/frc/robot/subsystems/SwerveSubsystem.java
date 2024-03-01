package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
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

import java.util.List;

import static frc.robot.Constants.NavigationConstants.DEFAULT_PATH_CONSTRAINTS;
import static frc.robot.Constants.ShootingConstants.POSE_HISTORY_DURATION;
import static frc.robot.Constants.Swerve.AZIMUTH_CONTROLLER_D;
import static frc.robot.Constants.Swerve.AZIMUTH_CONTROLLER_I;
import static frc.robot.Constants.Swerve.AZIMUTH_CONTROLLER_P;
import static frc.robot.Constants.Swerve.AZIMUTH_CONTROLLER_TOLERANCE;
import static frc.robot.Constants.Swerve.SWERVE_KINEMATICS;
import static frc.robot.Constants.Swerve.HOLONOMIC_PATH_FOLLOWER_CONFIG;

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveDrivePoseEstimator poseEstimator;
    public final SwerveModule[] swerveModules;
    public final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.Swerve.PIGEON_ID);
    public final VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator();
    private final Field2d field2d = new Field2d();
    private final TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(POSE_HISTORY_DURATION);
    private double previousTimestamp = 0;
    private final PIDController azimuthController = new PIDController(AZIMUTH_CONTROLLER_P, AZIMUTH_CONTROLLER_I, AZIMUTH_CONTROLLER_D);

    private double yawCorrection = 0;

    public SwerveSubsystem() {
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

        poseEstimator = new SwerveDrivePoseEstimator(SWERVE_KINEMATICS, getGyroYaw(), getModulePositions(), new Pose2d(new Translation2d(3, 3), new Rotation2d()));

        AutoBuilder.configureHolonomic(
                this::getPose,
                pose -> poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose),
                () -> SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates()),
                this::drive,
                HOLONOMIC_PATH_FOLLOWER_CONFIG,
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

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            translation = translation.unaryMinus();
        }

        if (fieldRelative) {
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroYaw()));
        } else {
            drive(new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        }
    }

    /**
     * Set the azimuth setpoint.
     *
     * @param azimuthSetpoint field-relative setpoint azimuth, not flipped by alliance
     * @see #driveWithAzimuth(Translation2d)
     */
    public void setAzimuthSetpoint(Rotation2d azimuthSetpoint) {
        SmartDashboard.putNumber("Azimuth Setpoint [deg]", azimuthSetpoint.getDegrees());
        azimuthController.setSetpoint(azimuthSetpoint.getRadians());
    }

    /**
     * Drive the robot whilst rotating it to achieve the goal azimuth
     *
     * @param translation field-relative translation, like in {@link #drive(Translation2d, double, boolean) drive}
     * @see #setAzimuthSetpoint(Rotation2d)
     */
    public void driveWithAzimuth(Translation2d translation) {
        drive(translation, yawCorrection, true);
    }

    /**
     * Drive the robot whilst rotating it to achieve the goal azimuth
     *
     * @param translation     field-relative translation, like in {@link #drive(Translation2d, double, boolean) drive}
     * @param azimuthSetpoint field-relative goal azimuth, not flipped by alliance
     * @see #setAzimuthSetpoint(Rotation2d)
     * @see #driveWithAzimuth(Translation2d)
     */
    public void driveWithAzimuth(Translation2d translation, Rotation2d azimuthSetpoint) {
        setAzimuthSetpoint(azimuthSetpoint);
        driveWithAzimuth(translation);
    }

    public boolean azimuthAtSetpoint() {
        return azimuthController.atSetpoint();
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
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
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

        updateByEstimator(visionPoseEstimator.estimateGlobalPoseFrontCam(poseEstimator.getEstimatedPosition()));
//        updateByEstimator(visionPoseEstimator.estimateGlobalPoseRearCam(poseEstimator.getEstimatedPosition()));

        field2d.setRobotPose(poseEstimator.getEstimatedPosition());

        SmartDashboard.putData("Field", field2d);

        // Calculate the azimuth control. Whilst it is always calculated, only
        // {@link #driveWithAzimuth driveWithAzimuth} uses it.
        SmartDashboard.putNumber("Azimuth [deg]", getPose().getRotation().getDegrees());
        yawCorrection = azimuthController.calculate(getPose().getRotation().getRadians());
        // new LinearSystem<>(null, null, null, null)
    }

    private void updateByEstimator(EstimatedRobotPose estimatedRobotPose) {
        EstimatedRobotPose visionRobotPose;

        if ((visionRobotPose = estimatedRobotPose) != null) {
            double currentTimestamp = visionRobotPose.timestampSeconds;

            if (previousTimestamp != currentTimestamp) {
                previousTimestamp = currentTimestamp;

                Pose3d visionPose = visionRobotPose.estimatedPose;
                poseEstimator.addVisionMeasurement(visionPose.toPose2d(), Timer.getFPGATimestamp());
            }
        }

        poseEstimator.update(getGyroYaw(), getModulePositions());
    }

    private void sampleRobotPose() {
        poseHistory.addSample(Timer.getFPGATimestamp(), getPose());
    }

    public TimeInterpolatableBuffer<Pose2d> getPoseHistory() {
        return poseHistory;
    }

    public void infrequentPeriodic() {
        sampleRobotPose();
    }

    public void stop() {
        drive(new ChassisSpeeds(0, 0, 0));
    }

    public void pathPlanToPose(Pose2d endPose) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(getPose(), endPose);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                DEFAULT_PATH_CONSTRAINTS, // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        AutoBuilder.followPath(path);
    }
}
