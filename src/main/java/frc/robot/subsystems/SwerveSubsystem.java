package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.vision.VisionPoseEstimator;
import org.photonvision.EstimatedRobotPose;

import java.util.List;

import static frc.robot.Constants.NavigationConstants.DEFAULT_PATH_CONSTRAINTS;
import static frc.robot.Constants.ShootingConstants.POSE_HISTORY_DURATION;
import static frc.robot.Constants.Swerve.SWERVE_KINEMATICS;
import static frc.robot.Constants.Swerve.holomonicPathFollowerConfig;

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveDrivePoseEstimator poseEstimator;
    public final SwerveModule[] swerveModules;
    public final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.Swerve.PIGEON_ID);
    public final VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator();
    private final Field2d field2d = new Field2d();
    private final TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(POSE_HISTORY_DURATION);
    private double previousTimestamp = 0;

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
                holomonicPathFollowerConfig,

                () -> {
                    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
                    return alliance == DriverStation.Alliance.Red;
                },

                this
        );
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        ChassisSpeeds chasesSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, poseEstimator.getEstimatedPosition().getRotation())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        drive(chasesSpeeds);
    }

    public void drive(Translation2d translation, double rotation) {
        Translation2d flippedTranslation = translation;

        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            flippedTranslation = translation.unaryMinus();
        }

        drive(flippedTranslation, rotation, true);
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
        gyro.setYaw(0);

    }

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

        /*for (SwerveModule mod : swerveModules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }*/

        EstimatedRobotPose visionRobotPose;

        if ((visionRobotPose = visionPoseEstimator.getEstimatedGlobalPose(getPose())) != null) {
            double currentTimestamp = visionRobotPose.timestampSeconds;

            if (previousTimestamp != currentTimestamp) {
                previousTimestamp = currentTimestamp;

                Pose3d visionPose = visionRobotPose.estimatedPose;
                poseEstimator.addVisionMeasurement(visionPose.toPose2d(), Timer.getFPGATimestamp());
            }
        }

        poseEstimator.update(getGyroYaw(), getModulePositions());
        field2d.setRobotPose(poseEstimator.getEstimatedPosition());

        SmartDashboard.putData("Field", field2d);
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

    public Command pathPlanToPose(Pose2d endPose) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(getPose(), endPose);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                DEFAULT_PATH_CONSTRAINTS, // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        return AutoBuilder.followPath(path);
    }
}
