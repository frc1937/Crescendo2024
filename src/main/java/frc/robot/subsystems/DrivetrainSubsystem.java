package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateHistory;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.SwerveModule5990;
import frc.robot.util.AllianceUtilities;
import frc.robot.poseestimation.PoseEstimator;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.VisionConstants.DEFAULT_POSE;

public class DrivetrainSubsystem extends SubsystemBase {
    private final Lock odometryLock = new ReentrantLock();
    private final PoseEstimator poseEstimator;
    private final SwerveModule5990[] swerveModules;
    private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.Swerve.PIGEON_ID);


    private final RobotStateHistory history = new RobotStateHistory();
    private double previousTimestamp = 0;

    private final ProfiledPIDController azimuthController = new ProfiledPIDController(
            AZIMUTH_CONTROLLER_P, AZIMUTH_CONTROLLER_I, AZIMUTH_CONTROLLER_D,
            AZIMUTH_CONTROLLER_CONSTRAINTS);

    private final double AZIMUTH_CONTROLLER_DEADBAND = 0.12;
    private final MedianFilter xMedianFilter = new MedianFilter(10);
    private final MedianFilter yMedianFilter = new MedianFilter(10);
    private final MedianFilter thetaMedianFilter = new MedianFilter(10);
    private double yawCorrection = 0;

    public DrivetrainSubsystem() {
        this.poseEstimator = null;

        gyro.configFactoryDefault();
        zeroGyro();

        swerveModules = new SwerveModule5990[]{
                new SwerveModule5990(Constants.Swerve.Module0.CONSTANTS, this),
                new SwerveModule5990(Constants.Swerve.Module1.CONSTANTS, this),
                new SwerveModule5990(Constants.Swerve.Module2.CONSTANTS, this),
                new SwerveModule5990(Constants.Swerve.Module3.CONSTANTS, this)
        };

        Timer.delay(1.0);

        configurePathPlanner();

        azimuthController.setTolerance(AZIMUTH_CONTROLLER_TOLERANCE);
        azimuthController.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putNumber("Azimuth Error [rad]", azimuthController.getPositionError());
        SmartDashboard.putNumber("Azimuth Current [deg]", getGyroAzimuth().getDegrees());
    }

    public DrivetrainSubsystem(PoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;

        swerveModules = new SwerveModule5990[]{
                new SwerveModule5990(Constants.Swerve.Module0.CONSTANTS, this),
                new SwerveModule5990(Constants.Swerve.Module1.CONSTANTS, this),
                new SwerveModule5990(Constants.Swerve.Module2.CONSTANTS, this),
                new SwerveModule5990(Constants.Swerve.Module3.CONSTANTS, this)
        };
    }

    public void lockSwerve() {
        //Point all modules inwards, set speed to 0
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];

        moduleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(315));
        moduleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        moduleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(225));
        moduleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(135));

        for (SwerveModule5990 mod : swerveModules) {
            mod.setDesiredState(moduleStates[mod.moduleNumber], false); //false so it doesn't check for speed < 0.01
        }
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean closedLoop) {
        ChassisSpeeds discretizedChassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(discretizedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule5990 mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], closedLoop);
        }
    }

    /**
     * Open loop drive for autonomous trajectory following commands
     *
     * @param chassisSpeeds - the speeds used
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        drive(chassisSpeeds, false);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean closedLoop) {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red)
            translation = translation.unaryMinus();

        if (fieldRelative) {
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getGyroAzimuth());
            drive(chassisSpeeds, closedLoop);
        } else {
            drive(new ChassisSpeeds(translation.getX(), translation.getY(), rotation), closedLoop);
        }
    }

    public void resetAzimuthController() {
        azimuthController.reset(getPose().toAlliancePose().getRotation().getRadians(), history.estimate().getVelocity().omegaRadiansPerSecond);
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
        driveWithAzimuth(translation, true);
    }

    public void driveWithAzimuth(Translation2d translation, boolean fieldRelative) {
        drive(translation, yawCorrection, fieldRelative, true);
    }
//todo: adapt DrivetrainSubsystem to use new localization system
    /**
     * Drive the robot whilst rotating it to achieve the goal azimuth
     *
     * @param translation field-relative translation, like in {@link #drive(Translation2d, double, boolean, boolean) drive}
     * @param azimuthGoal field-relative goal azimuth, not flipped by alliance
     * @see #setAzimuthGoal(Rotation2d)
     * @see #driveWithAzimuth(Translation2d)
     */
    public void driveWithAzimuth(Translation2d translation, Rotation2d azimuthGoal) {
        driveWithAzimuth(translation, azimuthGoal, true);
    }

    public void driveWithAzimuth(Translation2d translation, Rotation2d azimuthGoal, boolean fieldRelative) {
        setAzimuthGoal(azimuthGoal);
        driveWithAzimuth(translation, fieldRelative);
    }

    public boolean azimuthAtGoal(Measure<Angle> tolerance) {
        return Math.abs(azimuthController.getPositionError()) < tolerance.in(Radians);
    }

    public void resetPose() {
        poseEstimator.resetPose(DEFAULT_POSE);
    }

    public AllianceUtilities.AlliancePose2d getPose() {
        return poseEstimator.getCurrentPose();
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

    public void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }

    public void zeroGyro() {
        if(AllianceUtilities.isBlueAlliance()) {
            setHeading(Rotation2d.fromDegrees(0));
        } else {
            setHeading(Rotation2d.fromDegrees(180));
        }
    }

    public Rotation2d getGyroAzimuth() {
        return Constants.Swerve.INVERT_GYRO ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

//    public void resetModulesToAbsolute() {
//        for (SwerveModule mod : swerveModules) {
//            mod.resetToAbsolute();
//        }
//    } //This isn't needed. You already reset when initializing.

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro", gyro.getYaw());

        poseEstimator.updateFromOdometry(getGyroAzimuth(), getModulePositions());

        // Calculate the azimuth control. Whilst it is always calculated, only
        // {@link #driveWithAzimuth driveWithAzimuth} uses it.
        SmartDashboard.putNumber("swerve/azimuth [deg]", getPose().toAlliancePose().getRotation().getDegrees());
        SmartDashboard.putNumber("swerve/azimuth (gyro) [deg]", MathUtil.inputModulus(getGyroAzimuth().getDegrees(), -180, 180));
        SmartDashboard.putNumber("swerve/azimuth [target]", azimuthController.getGoal().position * 180 / Math.PI);

        yawCorrection = azimuthController.calculate(getPose().toAlliancePose().getRotation().getRadians());
        yawCorrection = MathUtil.applyDeadband(yawCorrection, AZIMUTH_CONTROLLER_DEADBAND);
    }

    public RobotStateHistory getHistory() {
        return history;
    }

    public void infrequentPeriodic() {
        sampleRobotPose();
    }

    public void stop() {
        drive(new ChassisSpeeds(0, 0, 0), false);
    }

    /**
     * Publish the current PID gains so they can be modified
     */
    public void publishControllerGains() {
        String path = "swerve/azimuth-controller";

        if (!SmartDashboard.containsKey(path+"/p")) {
            SmartDashboard.putNumber(path+"/p", AZIMUTH_CONTROLLER_P);
        }
        if (!SmartDashboard.containsKey(path + "/i")) {
            SmartDashboard.putNumber(path + "/i", AZIMUTH_CONTROLLER_I);
        }
        if (!SmartDashboard.containsKey(path + "/d")) {
            SmartDashboard.putNumber(path + "/d", AZIMUTH_CONTROLLER_D);
        }
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

    private void sampleRobotPose() {
        history.addSample(Timer.getFPGATimestamp(), getPose().toAlliancePose());
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> poseEstimator.getCurrentPose().toBlueAlliancePose(),
                pose -> poseEstimator.resetPose(AllianceUtilities.AlliancePose2d.fromAlliancePose(pose)), //TODO LOL THIS MIGHT BE RLY WRONG
                () -> SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates()),
                this::drive,
                AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                () -> !AllianceUtilities.isBlueAlliance(),
                this
        );
    }

    public Lock getOdometryLock() {
        return odometryLock;
    }
}
