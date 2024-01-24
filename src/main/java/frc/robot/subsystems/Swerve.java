package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

import java.io.IOException;
import java.util.Optional;

import static frc.robot.Constants.Swerve.SWERVE_KINEMATICS;
import static frc.robot.Constants.Swerve.holomonicPathFollowerConfig;

public class Swerve extends SubsystemBase {

    public final SwerveDrivePoseEstimator poseEstimator;
    public final SwerveModule[] swerveMods;
    public final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.Swerve.PIGEON_ID);
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonCamera photonCamera = new PhotonCamera("Photon1937");

    public Swerve()  {
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro", gyro.getYaw());

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }/*

        var pipelineResult = photonCamera.getLatestResult();
        var resultTimestamp = pipelineResult.getTimestampSeconds();

        if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
            previousPipelineTimestamp = resultTimestamp;
            var target = pipelineResult.getBestTarget();
            var fiducialId = target.getFiducialId();
            // Get the tag pose from field layout - consider that the layout will be null if it failed to load
            Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
            if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
                var targetPose = tagPose.get();
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

                var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
                poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
            }
        }*/
        // Update pose estimator with drivetrain sensors
        poseEstimator.update(getYaw(), getModulePositions());
    }

    public void stop() {
        drive(new Translation2d(), 0, false, false);
    }
}