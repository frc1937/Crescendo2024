package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.Controller;
import frc.robot.commands.AlignWithAmp;
import frc.robot.commands.AlignWithTag;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.commands.ShootToAmp;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TeleOpDrive;
import frc.robot.commands.calibration.FlywheelSysIdCharacterization;
import frc.robot.commands.calibration.GearRatioCharacterization;
import frc.robot.commands.calibration.MaxDrivetrainSpeedCharacterization;
import frc.robot.commands.calibration.MaxFlywheelSpeedCharacterization;
import frc.robot.commands.calibration.PitchSysIdCharacterization;
import frc.robot.commands.calibration.SysIdCharacterization;
import frc.robot.commands.calibration.WheelRadiusCharacterization;
import frc.robot.commands.leds.ColourByShooter;
import frc.robot.commands.leds.Flashing;
import frc.robot.poseestimation.PhotonCameraSource;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.poseestimation.PoseEstimator6328;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterPhysicsCalculations;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve5990;

import java.util.function.DoubleSupplier;

import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;
import static frc.lib.util.Controller.Axis.RIGHT_X;
import static frc.lib.util.Controller.Inputs.A;
import static frc.lib.util.Controller.Inputs.B;
import static frc.lib.util.Controller.Inputs.BACK;
import static frc.lib.util.Controller.Inputs.LEFT_BUMPER;
import static frc.lib.util.Controller.Inputs.RIGHT_BUMPER;
import static frc.lib.util.Controller.Inputs.START;
import static frc.lib.util.Controller.Inputs.X;
import static frc.lib.util.Controller.Inputs.Y;
import static frc.lib.util.Controller.Stick.LEFT_STICK;
import static frc.lib.util.Controller.Stick.RIGHT_STICK;
import static frc.robot.Constants.Transforms.ROBOT_TO_FRONT_CAMERA;
import static frc.robot.Constants.VisionConstants.FRONT_CAMERA_NAME;
import static frc.robot.subsystems.shooter.ShooterConstants.SPEAKER_BACK;
import static frc.robot.subsystems.shooter.ShooterConstants.SPEAKER_FRONT;

public class RobotContainer {
    private static final Controller driveController = new Controller(0);
    private static final Controller operatorController = new Controller(1);
    private final SendableChooser<Command> autoChooser;

    /* MAIN-DRIVER */
    private final Trigger drAButton = driveController.getButton(A);
    private final Trigger drYButton = driveController.getButton(Y);
    private final Trigger drBButton = driveController.getButton(B);
    private final Trigger drXButton = driveController.getButton(X);
    private final Trigger drStartButton = driveController.getButton(START);
    private final Trigger drBackButton = driveController.getButton(BACK);
    private final Trigger drLeftTrigger = driveController.getStick(LEFT_STICK);
    private final Trigger drRightTrigger = driveController.getStick(RIGHT_STICK);
    private final Trigger drLeftBumper = driveController.getButton(LEFT_BUMPER);
    private final Trigger drRightBumper = driveController.getButton(RIGHT_BUMPER);

    /* OPERATOR */
    private final Trigger opAButton = operatorController.getButton(A);
    private final Trigger opBButton = operatorController.getButton(B);
    private final Trigger opXButton = operatorController.getButton(X);
    /* Subsystems */
    private final PoseEstimator5990 poseEstimator5990;

    private final Swerve5990 swerve5990;

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final LEDsSubsystem leds = new LEDsSubsystem();
    /* Commands */
    private final ShooterCommands shooterCommands;
    private final ShooterPhysicsCalculations shooterPhysicsCalculations;
    private final AlignWithTag alignWithTag;
    private final Flashing flashingLEDs = new Flashing(leds);

    /* CONTROLS */
    private final Trigger hasNote = new Trigger(shooterSubsystem::isLoaded);

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        PoseEstimator6328 poseEstimator6328 = new PoseEstimator6328();

        PhotonCameraSource frontCamera = new PhotonCameraSource(FRONT_CAMERA_NAME, ROBOT_TO_FRONT_CAMERA);

        poseEstimator5990 = new PoseEstimator5990(poseEstimator6328,
                frontCamera
        );

        swerve5990 = new Swerve5990(poseEstimator5990);
        poseEstimator5990.setSwerve(swerve5990);

        shooterPhysicsCalculations = new ShooterPhysicsCalculations(shooterSubsystem, poseEstimator5990);
        shooterCommands = new ShooterCommands(shooterSubsystem, intakeSubsystem, leds, swerve5990, shooterPhysicsCalculations);

        alignWithTag = new AlignWithTag(swerve5990, poseEstimator5990, frontCamera);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }


    private void configureBindings() {
        hasNote.toggleOnTrue(new InstantCommand(() -> driveController.rumble(10, 2)));

        DoubleSupplier translationSup = () -> -driveController.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSup = () -> -driveController.getRawAxis(LEFT_X);
        DoubleSupplier rotationSup = () -> MathUtil.applyDeadband(-driveController.getRawAxis(RIGHT_X), Constants.STICK_DEADBAND);

        swerve5990.setDefaultCommand(
                new TeleOpDrive(
                        swerve5990,
                        translationSup,
                        strafeSup,
                        rotationSup,
                        () -> false
                )
        );

        leds.setDefaultCommand(new ColourByShooter(leds, shooterSubsystem));

        initializeButtons(translationSup, strafeSup, rotationSup, ButtonLayout.TELEOP);
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void infrequentPeriodic() {
        swerve5990.infrequentPeriodic();

//        if (RobotController.getBatteryVoltage() < 11.8) {
//            flashingLEDs.schedule();
//        }
    }

    public void frequentPeriodic() {
        poseEstimator5990.periodic();
        shooterPhysicsCalculations.feedRobotPose(poseEstimator5990.getCurrentPose().getBluePose());
    }

    private void initializeButtons(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, ButtonLayout layout) {
        switch (layout) {
            case PITCH_CHARACTERIZATION -> pitchCharacterizationLayout();
            case FLYWHEEL_CHARACTERIZATION -> flywheelCharacterizationLayout();
            case MAX_SPEEDS_CHARACTERIZATION -> maxSpeedsCharacterizationLayout(translationSup, strafeSup, rotationSup);
            case TELEOP -> teleopButtonsLayout(translationSup, strafeSup);
        }
    }

    private void teleopButtonsLayout(DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        drXButton.whileTrue(new ShootToAmp(shooterSubsystem, swerve5990, leds));
        drAButton.whileTrue(shooterCommands.shootPhysics(19));
        drBButton.whileTrue(new ShootOnTheMove(shooterSubsystem, poseEstimator5990, shooterCommands, swerve5990, translationSup, strafeSup, 16, shooterPhysicsCalculations));
        drYButton.whileTrue(new AlignWithAmp(swerve5990, translationSup, strafeSup));

        drLeftBumper.whileTrue(new AlignWithAmp(swerve5990, translationSup, strafeSup));
        drRightBumper.whileTrue(alignWithTag.driveToTag(10));

        drLeftTrigger.toggleOnFalse(shooterCommands.postIntake().withTimeout(0.65));
        drLeftTrigger.whileTrue((shooterCommands.floorIntake()));

        drRightTrigger.whileTrue(new IntakeCommands(intakeSubsystem).enableIntake(-0.9, true));

        drStartButton.onTrue(new InstantCommand(swerve5990::resetGyro));
        drBackButton.onTrue(new InstantCommand(swerve5990::lockSwerve)); //todo: this doesnt work

        //Operator buttons:
        opAButton.whileTrue(shooterCommands.shootNote(SPEAKER_FRONT));
        opBButton.whileTrue(shooterCommands.shootNote(SPEAKER_BACK));
    }

    private void flywheelCharacterizationLayout() {
        FlywheelSysIdCharacterization flywheelCharacterization = new FlywheelSysIdCharacterization(shooterSubsystem);
        characterizeByCharacterization(flywheelCharacterization);

        drLeftBumper.whileTrue(shooterCommands.setFlywheelSetpoint(14));

    }

    private void pitchCharacterizationLayout() {
        PitchSysIdCharacterization pitchCharacterization = new PitchSysIdCharacterization(shooterSubsystem);
        characterizeByCharacterization(pitchCharacterization);

        drLeftTrigger.whileTrue(shooterCommands.setPitchPosition(-10));
        drLeftBumper.whileTrue(shooterCommands.setPitchPosition(30));
        drRightTrigger.whileTrue(shooterCommands.setPitchPosition(75));
        drRightBumper.whileTrue(shooterCommands.setPitchPosition(100));

        drBackButton.whileTrue(new GearRatioCharacterization(shooterSubsystem.getPitch(), 0.5, 4));
    }

    private void maxSpeedsCharacterizationLayout(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        drAButton.whileTrue(new MaxDrivetrainSpeedCharacterization(swerve5990, translationSup, strafeSup, rotationSup, () -> false));//, 5.1m/s
        drBButton.whileTrue(new MaxFlywheelSpeedCharacterization(shooterSubsystem)); //5500 rpm flywheel

        drXButton.whileTrue(new WheelRadiusCharacterization(swerve5990, WheelRadiusCharacterization.Direction.CLOCKWISE));
        drYButton.whileTrue(new WheelRadiusCharacterization(swerve5990, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE));
    }

    private enum ButtonLayout {
        PITCH_CHARACTERIZATION,
        FLYWHEEL_CHARACTERIZATION,
        MAX_SPEEDS_CHARACTERIZATION,
        TELEOP
    }

    private void characterizeByCharacterization(SysIdCharacterization characterization) {
        SysIdRoutine.Direction forward = SysIdRoutine.Direction.kForward;
        SysIdRoutine.Direction reverse = SysIdRoutine.Direction.kReverse;

        drAButton.whileTrue(characterization.sysIdDynamicTest(forward));
        drBButton.whileTrue(characterization.sysIdDynamicTest(reverse));
        drYButton.whileTrue(characterization.sysIdQuastaticTest(forward));
        drXButton.whileTrue(characterization.sysIdQuastaticTest(reverse));
    }
}
