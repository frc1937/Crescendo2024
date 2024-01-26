package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.Optional;

import static frc.robot.Constants.Swerve.SWERVE_KINEMATICS;
import static frc.robot.Constants.Swerve.holomonicPathFollowerConfig;
import static frc.robot.Constants.VisionConstants.CAMERA_TO_ROBOT;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;

public class Swerve extends SubsystemBase {
    public final SwerveDrivePoseEstimator poseEstimator;
    public final SwerveModule[] swerveMods;
    public final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.Swerve.PIGEON_ID);
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonCamera photonCamera = new PhotonCamera("Photon1937");
    private double previousTimestamp = 0;
    private Pose3d aprilTagPos = new Pose3d();

    public Swerve() {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        gyro.configFactoryDefault();
        zeroGyro();

        swerveMods = new SwerveModule[]{
                new SwerveModule(0, Constants.Swerve.Mod0.CONSTANTS),
                new SwerveModule(1, Constants.Swerve.Mod1.CONSTANTS),
                new SwerveModule(2, Constants.Swerve.Mod2.CONSTANTS),
                new SwerveModule(3, Constants.Swerve.Mod3.CONSTANTS)
        };

        Timer.delay(1.0);
        resetModulesToAbsolute();

        poseEstimator = new SwerveDrivePoseEstimator(SWERVE_KINEMATICS, getYaw(), getModulePositions(), new Pose2d());
        resetOdometry(new Pose2d());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeed,
                this::drive,
                holomonicPathFollowerConfig,

                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },

                this
        );
    }

    public ChassisSpeeds getRobotRelativeSpeed() {
        return SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getChassisSpeedsFromValues(Translation2d translation, double rotation, boolean fieldRelative) {
        return fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(getChassisSpeedsFromValues(translation, rotation, fieldRelative));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }

        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public Pose3d getTagPose() {
        return aprilTagPos;
    }

    @Override
    public void periodic() {
        poseEstimator.update(getYaw(), getModulePositions());
        SmartDashboard.putNumber("Gyro", gyro.getYaw());

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        PhotonPipelineResult result = photonCamera.getLatestResult();
        double currentTimestamp = result.getTimestampSeconds();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Pose3d cameraPose = getPose3d().transformBy(ROBOT_TO_CAMERA);

            aprilTagPos = cameraPose.transformBy(cameraToTarget.inverse());

            SmartDashboard.putNumber("Target X: ", aprilTagPos.getX());
            SmartDashboard.putNumber("Target Y: ", aprilTagPos.getY());

            SmartDashboard.putNumber("CURR X: ", getPose().getX());
            SmartDashboard.putNumber("CURR Y: ", getPose().getY());
        }

        if (currentTimestamp != previousTimestamp && result.hasTargets()) {
            previousTimestamp = currentTimestamp;

            PhotonTrackedTarget target = result.getBestTarget();
            int fiducialId = target.getFiducialId();

            Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);

            if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
                Pose3d targetPose = tagPose.get();
                Transform3d cameraToTarget = target.getBestCameraToTarget();
                Pose3d cameraPose = targetPose.transformBy(cameraToTarget.inverse());

                Pose3d visionMeasurement = cameraPose.transformBy(CAMERA_TO_ROBOT);
               // poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), currentTimestamp);
            }
        }
    }

    public void stop() {
        drive(new Translation2d(), 0, false, false);
    }

    public Pose3d getPose3d() {
        Pose2d robotPose2d = getPose();

        return new Pose3d(
                robotPose2d.getX(),
                robotPose2d.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    }

    public boolean hasTargets() {
        return photonCamera.getLatestResult().hasTargets();
    }
}