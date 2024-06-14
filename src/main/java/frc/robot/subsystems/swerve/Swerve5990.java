package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.AlliancePose2d;
import frc.robot.poseestimation.PoseEstimator5990;

import java.util.stream.IntStream;

import static frc.lib.util.AlliancePose2d.AllianceUtils.fromCorrectPose;
import static frc.lib.util.AlliancePose2d.AllianceUtils.getCorrectRotation;
import static frc.lib.util.AlliancePose2d.AllianceUtils.isBlueAlliance;
import static frc.robot.Constants.CanIDConstants.PIGEON_ID;
import static frc.robot.Constants.DRIVE_NEUTRAL_DEADBAND;
import static frc.robot.Constants.ROTATION_NEUTRAL_DEADBAND;
import static frc.robot.Constants.Transforms.ROBOT_TO_FRONT_CAMERA;
import static frc.robot.subsystems.swerve.SwerveConstants.AZIMUTH_CONTROLLER_TOLERANCE_DEG;
import static frc.robot.subsystems.swerve.SwerveConstants.AZIMUTH_MAX_VELOCITY;
import static frc.robot.subsystems.swerve.SwerveConstants.AutoConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG;
import static frc.robot.subsystems.swerve.SwerveConstants.INVERT_GYRO;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve.SwerveConstants.Module0;
import static frc.robot.subsystems.swerve.SwerveConstants.Module1;
import static frc.robot.subsystems.swerve.SwerveConstants.Module2;
import static frc.robot.subsystems.swerve.SwerveConstants.Module3;
import static frc.robot.subsystems.swerve.SwerveConstants.NUMBER_OF_MODULES;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_KINEMATICS;

public class Swerve5990 extends SubsystemBase {
    private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(PIGEON_ID);

    private final PoseEstimator5990 poseEstimator5990;
    private final SwerveModule5990[] modules;

    private final ProfiledPIDController azimuthController =
            new ProfiledPIDController(0.1, 0, 0,
                    new TrapezoidProfile.Constraints(720, 720));

    private int azimuthAtReferenceCount = 0;

    private final StructArrayPublisher<SwerveModuleState> currentStates = NetworkTableInstance.getDefault().getStructArrayTopic("CurrentStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> targetStates = NetworkTableInstance.getDefault().getStructArrayTopic("TargetStates", SwerveModuleState.struct).publish();

    private final StructArrayPublisher<Pose3d> cameraPoses = NetworkTableInstance.getDefault().getStructArrayTopic("CameraPoses", Pose3d.struct).publish();

    public Swerve5990(PoseEstimator5990 poseEstimator5990) {
        this.poseEstimator5990 = poseEstimator5990;

        gyro.configFactoryDefault();
        modules = getModules();

        azimuthController.setTolerance(AZIMUTH_CONTROLLER_TOLERANCE_DEG.get());

        resetAzimuthController();
        azimuthController.enableContinuousInput(-180, 180);

        configurePathPlanner();
    }

    @Override
    public void periodic() {
        if (azimuthController.atGoal()) azimuthAtReferenceCount++;
        else azimuthAtReferenceCount = 0;

        //logging data n shit
        currentStates.set(getModuleStates());
        targetStates.set(getModuleTargetStates());

        Pose3d robotPose = new Pose3d(poseEstimator5990.getCurrentPose().getCorrectPose());

        cameraPoses.set(
                new Pose3d[]{
                        robotPose.plus(ROBOT_TO_FRONT_CAMERA)

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

    public void resetAzimuthController() {
        azimuthController.reset(poseEstimator5990.getCurrentPose().getBluePose().getRotation().getDegrees());
    }

    public Rotation2d getGyroAzimuth() {
        //Ensure gyro is always -180 to 180
        double currentYaw = MathUtil.inputModulus(gyro.getYaw(), -180, 180);

        return INVERT_GYRO ? Rotation2d.fromDegrees(360 - currentYaw) : Rotation2d.fromDegrees(currentYaw);
    }

    public void stop() {
        for (SwerveModule5990 mod : modules) {
            mod.stop();
        }
    }

    public void resetGyro() {
        final AlliancePose2d currentPose = poseEstimator5990.getCurrentPose();
        final AlliancePose2d resetPose = AlliancePose2d.AllianceUtils.fromBluePose(
                new Pose2d(
                        currentPose.getBluePose().getTranslation(),
                        new Rotation2d(0)
                )
        );
        poseEstimator5990.resetPose(resetPose);
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
     * Get the position of all drive wheels in radians.
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
        Rotation2d currentAngle = poseEstimator5990.getCurrentPose().getBluePose().getRotation();
        double omegaRadiansPerSecond = azimuthController.calculate(currentAngle.getDegrees(), targetAngle.getDegrees());

        SmartDashboard.putNumber("Omega/Speed radPerSec azimuth ", omegaRadiansPerSecond);
        SmartDashboard.putNumber("Omega/Speed Target Azimuth [DEG]", targetAngle.getDegrees());
        SmartDashboard.putNumber("Omega/Speed Current Angle [DEG Gyro]", currentAngle.getDegrees());

        return omegaRadiansPerSecond;
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
