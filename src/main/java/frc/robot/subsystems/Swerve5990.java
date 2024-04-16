package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateHistory;
import frc.robot.Constants;
import frc.robot.SwerveModule5990;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.util.AllianceUtilities;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.DRIVE_NEUTRAL_DEADBAND;
import static frc.robot.Constants.ROTATION_NEUTRAL_DEADBAND;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.Swerve.AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG;

public class Swerve5990 extends SubsystemBase {
    private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(Constants.Swerve.PIGEON_ID);

    private final PoseEstimator5990 poseEstimator5990;
    private final SwerveModule5990[] modules;

    private final RobotStateHistory history = new RobotStateHistory();

    private final ProfiledPIDController azimuthController = new ProfiledPIDController(
            AZIMUTH_CONTROLLER_P, AZIMUTH_CONTROLLER_I, AZIMUTH_CONTROLLER_D,
            AZIMUTH_CONTROLLER_CONSTRAINTS);

    private final ProfiledPIDController translationController = new ProfiledPIDController(
            TRANSLATION_CONTROLLER_P, 0, 0,
            new TrapezoidProfile.Constraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION)
    );

    public Swerve5990(PoseEstimator5990 poseEstimator5990) {
        this.poseEstimator5990 = poseEstimator5990;

        gyro.configFactoryDefault();
        resetGyro();

        modules = getModules();

        Timer.delay(0.1); //todo: Check without this, if this is even needed

        configurePathPlanner();
        setupAzimuthController();
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

    public Rotation2d getGyroAzimuth() {
        return Constants.Swerve.INVERT_GYRO ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void stop() {
        for (SwerveModule5990 mod : modules) {
            mod.stop();
        }
    }

    public void resetGyro() {
        Pose2d pose = new Pose2d(0, 0, new Rotation2d());
        gyro.setYaw(AllianceUtilities.AlliancePose2d.fromBlueAlliancePose(pose).toAlliancePose().getRotation().getDegrees());
    }

    public SwerveDriveWheelPositions getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (SwerveModule5990 mod : modules)
            positions[mod.swerveModuleConstants.moduleNumber] = mod.getCurrentPosition();

        return new SwerveDriveWheelPositions(positions);
    }

    public RobotStateHistory getHistory() {
        return history;
    }

    public void infrequentPeriodic() {
        sampleRobotPose();
    }

    public void setupAzimuthController() {
        azimuthController.reset(poseEstimator5990.getCurrentPose().toBlueAlliancePose().getRotation().getRadians());

        azimuthController.setTolerance(AZIMUTH_CONTROLLER_TOLERANCE);
        azimuthController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public boolean azimuthAtGoal(Measure<Angle> tolerance) {
        return Math.abs(azimuthController.getPositionError()) < tolerance.in(Radians);
    }

    public void drive(double xPower, double yPower, double thetaPower, boolean robotCentric) {
        if(robotCentric) {
            driveSelfRelative(xPower, yPower, thetaPower);
        } else {
            driveFieldRelative(xPower, yPower, thetaPower);
        }
    }

    /**
     *
     * @param targetPose - blue alliance form
     */
    public void PIDToPose(Pose2d targetPose) {
        Pose2d currentPose = poseEstimator5990.getCurrentPose().toBlueAlliancePose();

        double xSpeed = translationController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = translationController.calculate(currentPose.getY(), targetPose.getY());

        int direction = AllianceUtilities.isBlueAlliance() ? 1 : -1;

        ChassisSpeeds speeds = new ChassisSpeeds(
                xSpeed * direction,
                ySpeed * direction,
                calculateProfiledSpeedToAngle(targetPose.getRotation())
        );

        driveSelfRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, currentPose.getRotation()));
    }

    public void driveFieldRelative(double xPower, double yPower, Rotation2d targetAngle) {
        targetAngle = AllianceUtilities.toMirroredAllianceRotation(targetAngle);

        Rotation2d currentAngle = poseEstimator5990.getCurrentPose().toAlliancePose().getRotation();

        ChassisSpeeds selfRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(powersToSpeeds(xPower, yPower, 0), currentAngle);

        selfRelativeSpeeds.omegaRadiansPerSecond = calculateProfiledSpeedToAngle(targetAngle);

        driveSelfRelative(selfRelativeSpeeds);
    }

    public void driveFieldRelative(double xPower, double yPower, double thetaPower) {
        ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, thetaPower);
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, poseEstimator5990.getCurrentPose().toAlliancePose().getRotation());

        driveSelfRelative(speeds);
    }

    public void driveSelfRelative(double xPower, double yPower, double thetaPower) {
        driveSelfRelative(powersToSpeeds(xPower, yPower, thetaPower));
    }

    public void driveSelfRelative(double xPower, double yPower, Rotation2d targetAngle) {
        ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledSpeedToAngle(targetAngle);

        driveSelfRelative(speeds);
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

    private void driveSelfRelative(ChassisSpeeds speeds) {
        ChassisSpeeds discretizedChassisSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

        if(isStill(discretizedChassisSpeeds)) {
            stop();
            return;
        }

        SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(discretizedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule5990 mod : modules) {
            mod.setTargetState(swerveModuleStates[mod.swerveModuleConstants.moduleNumber], true);
        }
    }


    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * MAX_SPEED,
                yPower * MAX_SPEED,
                Math.pow(thetaPower, 2) * Math.signum(thetaPower) * MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
        );
    }


    private double calculateProfiledSpeedToAngle(Rotation2d angle) {
        double currentAngle = poseEstimator5990.getCurrentPose().toAlliancePose().getRotation().getRadians();
        double yawCorrection = azimuthController.calculate(
                currentAngle,
                angle.getRadians()
        );

        yawCorrection = MathUtil.applyDeadband(yawCorrection, AZIMUTH_CONTROLLER_DEADBAND);

        return yawCorrection;
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> poseEstimator5990.getCurrentPose().toBlueAlliancePose(),
                pose -> poseEstimator5990.resetPose(AllianceUtilities.AlliancePose2d.fromAlliancePose(pose)),
                //TODO LOL THIS MIGHT BE RLY WRONG

                this::getSelfRelativeVelocity,
                this::driveSelfRelative,

                HOLONOMIC_PATH_FOLLOWER_CONFIG,
                () -> !AllianceUtilities.isBlueAlliance(),
                this
        );
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (SwerveModule5990 mod : modules) {
            states[mod.swerveModuleConstants.moduleNumber] = mod.getCurrentState(); //TODO: Make a getState func
        }

        return states;
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

    private SwerveModule5990[] getModules() {
        return new SwerveModule5990[]{
                new SwerveModule5990(Constants.Swerve.Module0.CONSTANTS),
                new SwerveModule5990(Constants.Swerve.Module1.CONSTANTS),
                new SwerveModule5990(Constants.Swerve.Module2.CONSTANTS),
                new SwerveModule5990(Constants.Swerve.Module3.CONSTANTS)
        };
    }

    private void sampleRobotPose() {
        history.addSample(Timer.getFPGATimestamp(), poseEstimator5990.getCurrentPose().toAlliancePose());
    }
}
