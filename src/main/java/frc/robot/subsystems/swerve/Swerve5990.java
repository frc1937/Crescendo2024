package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.poseestimation.PoseEstimator5990;

import java.util.stream.IntStream;

import static frc.lib.util.AlliancePose2d.AllianceUtils.*;
import static frc.robot.Constants.CanIDConstants.PIGEON_ID;
import static frc.robot.Constants.DRIVE_NEUTRAL_DEADBAND;
import static frc.robot.Constants.ROTATION_NEUTRAL_DEADBAND;
import static frc.robot.Constants.Transforms.ROBOT_TO_FRONT_CAMERA;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG;

public class Swerve5990 extends SubsystemBase {
    private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(PIGEON_ID);

    private final PoseEstimator5990 poseEstimator5990;
    private final SwerveModule5990[] modules;

    private final HolonomicDriveController driveController =
            new HolonomicDriveController(
                    new PIDController(TRANSLATION_CONTROLLER_P, 0, 0),
                    new PIDController(TRANSLATION_CONTROLLER_P, 0, 0),
                    new ProfiledPIDController(AZIMUTH_CONTROLLER_KP.get(), 0, 0,
                            new TrapezoidProfile.Constraints(AZIMUTH_MAX_VELOCITY.get(), AZIMUTH_MAX_ACCELERATION.get()))
            );

    private int azimuthAtReferenceCount = 0;

    private final StructArrayPublisher<SwerveModuleState> currentStates = NetworkTableInstance.getDefault().getStructArrayTopic("CurrentStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> targetStates = NetworkTableInstance.getDefault().getStructArrayTopic("TargetStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<Pose3d> cameraPoses = NetworkTableInstance.getDefault().getStructArrayTopic("CameraPoses", Pose3d.struct).publish();

    public Swerve5990(PoseEstimator5990 poseEstimator5990) {
        this.poseEstimator5990 = poseEstimator5990;

        gyro.configFactoryDefault();
        resetGyro();

        modules = getModules();

        driveController.setTolerance(new Pose2d(0.2, 0.2, Rotation2d.fromDegrees(AZIMUTH_CONTROLLER_TOLERANCE_DEG.get())));

        initializeAzimuthController();
        configurePathPlanner();
    }

    private void initializeAzimuthController() {
        driveController.getThetaController().reset(getGyroAzimuth().getRadians());
        driveController.getThetaController().enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        if (driveController.atReference()) azimuthAtReferenceCount++;
        else azimuthAtReferenceCount = 0;

        //logging data n shit
        currentStates.set(getModuleStates());
        targetStates.set(getModuleTargetStates());

        Pose3d robotPose = new Pose3d(poseEstimator5990.getCurrentPose().getCorrectPose());

        cameraPoses.set(
                new Pose3d[]{
                        robotPose.plus(ROBOT_TO_FRONT_CAMERA.inverse())
                }
        );

        for (int i = 0; i < NUMBER_OF_MODULES; i++) {
            modules[i].periodic();
        }
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
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
        gyro.setYaw(0);
    }

    public SwerveDriveWheelPositions getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[NUMBER_OF_MODULES];

        for (SwerveModule5990 mod : modules)
            positions[mod.swerveModuleConstants.moduleNumber()] = mod.getCurrentPosition();

        return new SwerveDriveWheelPositions(positions);
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

    public void driveFieldRelative(double xPower, double yPower, Rotation2d targetAngle) {
        targetAngle = getCorrectRotation(targetAngle);

        Rotation2d currentAngle = poseEstimator5990.getCurrentPose().getBluePose().getRotation();
        ChassisSpeeds selfRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(powersToSpeeds(xPower, yPower, 0), currentAngle);

        selfRelativeSpeeds.omegaRadiansPerSecond = determineProfiledSpeedToAngle(targetAngle);

        driveSelfRelative(selfRelativeSpeeds);
    }

    public void driveToPose(Pose2d targetPose) {
        ChassisSpeeds speeds = driveController.calculate(poseEstimator5990.getCurrentPose().getBluePose(),
                targetPose, 0.3,
                targetPose.getRotation());

        driveSelfRelative(speeds);
    }

    public boolean azimuthAtGoal() {
        return azimuthAtReferenceCount > 3;
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
        speeds.omegaRadiansPerSecond = determineProfiledSpeedToAngle(targetAngle);

        driveSelfRelative(speeds);
    }

    /**
     * Get the position of all drive wheels in meters.
     */
    public double[] getWheelPositions() {
        return IntStream.range(0, NUMBER_OF_MODULES).mapToDouble(i -> modules[i].getWheelDistanceTraveledRadians()).toArray();
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

    public void driveSelfRelative(ChassisSpeeds speeds) {
        ChassisSpeeds discretizedChassisSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

        if (isStill(discretizedChassisSpeeds)) {
            stop();
            return;
        }

        SwerveModuleState[] swerveModuleStates = SWERVE_KINEMATICS.toSwerveModuleStates(discretizedChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED_MPS);

        for (SwerveModule5990 mod : modules) {
            mod.setTargetState(swerveModuleStates[mod.swerveModuleConstants.moduleNumber()], false);
        }
    }

    public void lockSwerve() {
        final SwerveModuleState
                right = new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                left = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

        ParallelRaceGroup functionalCommand = new FunctionalCommand(
                () -> {
                },
                () -> {
                    modules[0].setTargetState(left, false);
                    modules[1].setTargetState(right, false);
                    modules[2].setTargetState(right, false);
                    modules[3].setTargetState(left, false);
                },
                interrupt -> {
                },
                () -> false,
                this
        ).withTimeout(0.3);

        functionalCommand.schedule();
    }

    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * MAX_SPEED_MPS,
                yPower * MAX_SPEED_MPS,
                Math.pow(thetaPower, 2) * Math.signum(thetaPower) * AZIMUTH_MAX_VELOCITY.get()
        );
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
        SwerveModuleState[] states = new SwerveModuleState[NUMBER_OF_MODULES];

        for (SwerveModule5990 mod : modules) {
            states[mod.swerveModuleConstants.moduleNumber()] = mod.getCurrentState();
        }

        return states;
    }

    private SwerveModuleState[] getModuleTargetStates() {
        SwerveModuleState[] states = new SwerveModuleState[NUMBER_OF_MODULES];

        for (SwerveModule5990 mod : modules) {
            states[mod.swerveModuleConstants.moduleNumber()] = mod.getTargetState();
        }

        return states;
    }

    private double determineProfiledSpeedToAngle(Rotation2d targetAngle) {
        Rotation2d currentAngle = getGyroAzimuth();

        ChassisSpeeds chassisSpeeds = driveController.calculate(
                new Pose2d(0, 0, currentAngle),
                new Pose2d(0, 0, targetAngle),
                0,
                targetAngle
        ); //this returns the speeds in radians. Do everything in radians, therefore.

        double omegaSpeedRadiansPerSecond = MathUtil.applyDeadband(chassisSpeeds.omegaRadiansPerSecond, AZIMUTH_CONTROLLER_DEADBAND);

        SmartDashboard.putNumber("Omega/Speed RadPerSec Azimuth", omegaSpeedRadiansPerSecond);
        SmartDashboard.putNumber("Omega/Speed Target Azimuth [DEG]", targetAngle.getDegrees());
        SmartDashboard.putNumber("Omega/Speed Current Angle [DEG Gyro]", currentAngle.getDegrees());

        return omegaSpeedRadiansPerSecond;
    }

    private SwerveModule5990[] getModules() {
        return new SwerveModule5990[]{
                new SwerveModule5990(Module0.CONSTANTS),
                new SwerveModule5990(Module1.CONSTANTS),
                new SwerveModule5990(Module2.CONSTANTS),
                new SwerveModule5990(Module3.CONSTANTS)
        };
    }
}
