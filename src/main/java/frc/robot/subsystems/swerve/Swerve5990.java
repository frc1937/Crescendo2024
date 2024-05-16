package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.RobotStateHistory;
import frc.robot.poseestimation.PoseEstimator5990;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.Constants.CanIDConstants.PIGEON_ID;
import static frc.robot.Constants.DRIVE_NEUTRAL_DEADBAND;
import static frc.robot.Constants.ROTATION_NEUTRAL_DEADBAND;
import static frc.robot.subsystems.swerve.SwerveConstants.AZIMUTH_CONTROLLER_CONSTRAINTS;
import static frc.robot.subsystems.swerve.SwerveConstants.AZIMUTH_CONTROLLER_D;
import static frc.robot.subsystems.swerve.SwerveConstants.AZIMUTH_CONTROLLER_DEADBAND;
import static frc.robot.subsystems.swerve.SwerveConstants.AZIMUTH_CONTROLLER_I;
import static frc.robot.subsystems.swerve.SwerveConstants.AZIMUTH_CONTROLLER_P;
import static frc.robot.subsystems.swerve.SwerveConstants.AZIMUTH_CONTROLLER_TOLERANCE;
import static frc.robot.subsystems.swerve.SwerveConstants.AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG;
import static frc.robot.subsystems.swerve.SwerveConstants.INVERT_GYRO;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_SPEED;
import static frc.robot.subsystems.swerve.SwerveConstants.Module0;
import static frc.robot.subsystems.swerve.SwerveConstants.Module1;
import static frc.robot.subsystems.swerve.SwerveConstants.Module2;
import static frc.robot.subsystems.swerve.SwerveConstants.Module3;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_KINEMATICS;
import static frc.robot.subsystems.swerve.SwerveConstants.TRANSLATION_CONTROLLER_P;
import static frc.robot.subsystems.swerve.SwerveConstants.TRANSLATION_MAX_ACCELERATION;
import static frc.robot.subsystems.swerve.SwerveConstants.TRANSLATION_MAX_VELOCITY;
import static frc.lib.util.AlliancePose2d.AllianceUtils.fromBluePose;
import static frc.lib.util.AlliancePose2d.AllianceUtils.fromCorrectPose;
import static frc.lib.util.AlliancePose2d.AllianceUtils.getCorrectRotation;
import static frc.lib.util.AlliancePose2d.AllianceUtils.isBlueAlliance;

public class Swerve5990 extends SubsystemBase {
    private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(PIGEON_ID);

    private final PoseEstimator5990 poseEstimator5990;
    private final SwerveModule5990[] modules;

    private final RobotStateHistory stateHistory = new RobotStateHistory();

    private final ProfiledPIDController azimuthController = new ProfiledPIDController(
            AZIMUTH_CONTROLLER_P, AZIMUTH_CONTROLLER_I, AZIMUTH_CONTROLLER_D,
            AZIMUTH_CONTROLLER_CONSTRAINTS);

    private final ProfiledPIDController translationController = new ProfiledPIDController(
            TRANSLATION_CONTROLLER_P, 0, 0,
            new TrapezoidProfile.Constraints(TRANSLATION_MAX_VELOCITY, TRANSLATION_MAX_ACCELERATION)
    );

    private final StructArrayPublisher<SwerveModuleState> currentStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("CurrentStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> targetStates = NetworkTableInstance.getDefault()
            .getStructArrayTopic("TargetStates", SwerveModuleState.struct).publish();

    public Swerve5990(PoseEstimator5990 poseEstimator5990) {
        this.poseEstimator5990 = poseEstimator5990;

        gyro.configFactoryDefault();
        resetGyro();

        modules = getModules();

        setupAzimuthController();
        configurePathPlanner();
    }

    @Override
    public void periodic() {
        //logging data n shit
        currentStates.set(getModuleStates());
        targetStates.set(getModuleTargetStates());
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
        return INVERT_GYRO ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void stop() {
        for (SwerveModule5990 mod : modules) {
            mod.stop();
        }
    }

    public void resetGyro() {
        Pose2d pose = new Pose2d(0, 0, new Rotation2d());
        gyro.setYaw(fromBluePose(pose).getCorrectPose().getRotation().getDegrees());
    }

    public SwerveDriveWheelPositions getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (SwerveModule5990 mod : modules)
            positions[mod.swerveModuleConstants.moduleNumber()] = mod.getCurrentPosition();

        return new SwerveDriveWheelPositions(positions);
    }

    public RobotStateHistory getStateHistory() {
        return stateHistory;
    }

    public void infrequentPeriodic() {
        sampleRobotPose();
    }

    private void setupAzimuthController() {
        azimuthController.reset(poseEstimator5990.getCurrentPose().getBluePose().getRotation().getRadians());

        azimuthController.setTolerance(AZIMUTH_CONTROLLER_TOLERANCE);
        azimuthController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public boolean azimuthAtGoal(Measure<Angle> tolerance) {
        return Math.abs(azimuthController.getPositionError()) < tolerance.in(Radians);
    }

    /**
     * Drive the robot according to the given inputs.
     *
     * @param xPower       - the translation power
     * @param yPower       - the strafe power
     * @param thetaPower   - the rotation power
     * @param robotCentric - whether the robot should drive relative to itself or the field
     */

    public void drive(double xPower, double yPower, double thetaPower, boolean robotCentric) {
        if (robotCentric) {
            driveSelfRelative(xPower, yPower, thetaPower);
        } else {
            driveFieldRelative(xPower, yPower, thetaPower);
        }
    }

    /**
     * Drive the robot whilst rotating to a specific angle.
     * @param xPower - the translation power
     * @param yPower - the strafe power
     * @param targetAzimuthAngle - the angle to rotate to
     */
    public void driveWithTargetAzimuth(double xPower, double yPower, Rotation2d targetAzimuthAngle) {
        driveFieldRelative(xPower, yPower, calculateProfiledSpeedToAngle(targetAzimuthAngle));
        //todo: check if works
    }

    /**
     * @param targetPose - blue alliance form
     */
    public void pidToPose(Pose2d targetPose) {
        Pose2d currentPose = poseEstimator5990.getCurrentPose().getBluePose();

        double xSpeed = translationController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = translationController.calculate(currentPose.getY(), targetPose.getY());

        int direction = isBlueAlliance() ? 1 : -1;

        ChassisSpeeds speeds = new ChassisSpeeds(
                xSpeed * direction,
                ySpeed * direction,
                calculateProfiledSpeedToAngle(targetPose.getRotation())
        );

        driveSelfRelative(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, currentPose.getRotation()));
    }

    public void driveFieldRelative(double xPower, double yPower, Rotation2d targetAngle) {
        targetAngle = getCorrectRotation(targetAngle);

        Rotation2d currentAngle = poseEstimator5990.getCurrentPose().getCorrectPose().getRotation();

        ChassisSpeeds selfRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(powersToSpeeds(xPower, yPower, 0), currentAngle);

        selfRelativeSpeeds.omegaRadiansPerSecond = calculateProfiledSpeedToAngle(targetAngle);

        driveSelfRelative(selfRelativeSpeeds);
    }

    public void driveFieldRelative(double xPower, double yPower, double thetaPower) {
        ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, thetaPower);
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, poseEstimator5990.getCurrentPose().getCorrectPose().getRotation());

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

        if (!SmartDashboard.containsKey(path + "/p")) {
            SmartDashboard.putNumber(path + "/p", AZIMUTH_CONTROLLER_P);
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

    /**
     * Returns whether the given chassis speeds are considered to be "still" by the swerve neutral deadband.
     *
     * @param chassisSpeeds the chassis speeds to check
     * @return true if the chassis speeds are considered to be "still"
     */
    public boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) <= DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= ROTATION_NEUTRAL_DEADBAND;
    }

    private void driveSelfRelative(ChassisSpeeds speeds) {
        ChassisSpeeds discretizedChassisSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

        if (isStill(discretizedChassisSpeeds)) {
            stop();
            return;
        }

        SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(discretizedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        for (SwerveModule5990 mod : modules) {
            mod.setTargetState(swerveModuleStates[mod.swerveModuleConstants.moduleNumber()], false);
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
        double currentAngle = poseEstimator5990.getCurrentPose().getCorrectPose().getRotation().getRadians();
        double yawCorrection = azimuthController.calculate(
                currentAngle,
                angle.getRadians()
        );

        yawCorrection = MathUtil.applyDeadband(yawCorrection, AZIMUTH_CONTROLLER_DEADBAND);

        return yawCorrection;
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> poseEstimator5990.getCurrentPose().getBluePose(),
                pose -> poseEstimator5990.resetPose(fromCorrectPose(pose)),

                this::getSelfRelativeVelocity,
                this::driveSelfRelative,

                HOLONOMIC_PATH_FOLLOWER_CONFIG,
                () -> !isBlueAlliance(),
                this
        );
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (SwerveModule5990 mod : modules) {
            states[mod.swerveModuleConstants.moduleNumber()] = mod.getCurrentState();
        }

        return states;
    }

    private SwerveModuleState[] getModuleTargetStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (SwerveModule5990 mod : modules) {
            states[mod.swerveModuleConstants.moduleNumber()] = mod.getTargetState();
        }

        return states;
    }



    private SwerveModule5990[] getModules() {
        return new SwerveModule5990[]{
                new SwerveModule5990(Module0.CONSTANTS),
                new SwerveModule5990(Module1.CONSTANTS),
                new SwerveModule5990(Module2.CONSTANTS),
                new SwerveModule5990(Module3.CONSTANTS)
        };
    }

    private void sampleRobotPose() {
        stateHistory.addSample(Timer.getFPGATimestamp(), poseEstimator5990.getCurrentPose().getCorrectPose());
    }
}
