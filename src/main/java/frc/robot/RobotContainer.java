package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.Controller;
import frc.robot.commands.AlignWithAmp;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.MountCommand;
import frc.robot.commands.PrepareShooter;
import frc.robot.commands.ShootOnTheMove;
import frc.robot.commands.ShootToAmp;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterKick;
import frc.robot.commands.TeleOpDrive;
import frc.robot.commands.calibration.MaxDrivetrainSpeedCharacterization;
import frc.robot.commands.calibration.MaxFlywheelSpeedCharacterization;
import frc.robot.commands.calibration.PitchCharacterization;
import frc.robot.commands.leds.ColourByShooter;
import frc.robot.poseestimation.PhotonCameraSource;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.poseestimation.PoseEstimator6328;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.mount.MountSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve5990;

import java.util.function.DoubleSupplier;

import static frc.lib.math.Conversions.tangentialVelocityFromRPM;
import static frc.lib.util.Controller.Axis.LEFT_X;
import static frc.lib.util.Controller.Axis.LEFT_Y;
import static frc.lib.util.Controller.Axis.RIGHT_X;
import static frc.lib.util.Controller.Axis.RIGHT_Y;
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
import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTING_DELAY;
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
    private final MountSubsystem mountSubsystem = new MountSubsystem();
    private final LEDsSubsystem leds = new LEDsSubsystem();
    /* Commands */
    private final ShooterCommands shooterCommands;


    /* CONTROLS */
    private final Trigger hasNote = new Trigger(shooterSubsystem::isLoaded);

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        PoseEstimator6328 poseEstimator6328 = new PoseEstimator6328();

        poseEstimator5990 = new PoseEstimator5990(poseEstimator6328,
                new PhotonCameraSource(FRONT_CAMERA_NAME, ROBOT_TO_FRONT_CAMERA)
        );

        swerve5990 = new Swerve5990(poseEstimator5990);
        poseEstimator5990.setSwerve(swerve5990);

        shooterCommands = new ShooterCommands(shooterSubsystem, intakeSubsystem, leds, poseEstimator5990);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        registerCommands();
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

        mountSubsystem.setDefaultCommand(
                new MountCommand(mountSubsystem,
                        () -> MathUtil.applyDeadband(-operatorController.getRawAxis(LEFT_Y), Constants.STICK_DEADBAND * 0.5),
                        () -> MathUtil.applyDeadband(-operatorController.getRawAxis(RIGHT_Y), Constants.STICK_DEADBAND * 0.5)
                )
        );

        initializeButtons(translationSup, strafeSup, rotationSup, ButtonLayout.TELEOP);
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void infrequentPeriodic() {
        swerve5990.infrequentPeriodic();
    }

    public void frequentPeriodic() {
        poseEstimator5990.periodic();
    }

    private void registerCommands() {
        NamedCommands.registerCommand("Intake", shooterCommands.floorIntake().withTimeout(2));
        NamedCommands.registerCommand("PostIntake", shooterCommands.postIntake());
        NamedCommands.registerCommand("IntakeUnicorn", shooterCommands.floorIntake().withTimeout(2.7));

        NamedCommands.registerCommand("ShooterKick", new ShooterKick(shooterSubsystem).withTimeout(SHOOTING_DELAY));

        double flywheelDiameter = Units.inchesToMeters(4);

        NamedCommands.registerCommand("AdjustShooter1", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(108.25), tangentialVelocityFromRPM(3000, flywheelDiameter))));
        NamedCommands.registerCommand("AdjustShooter2", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(114), tangentialVelocityFromRPM(3000, flywheelDiameter))));
        NamedCommands.registerCommand("AdjustShooter3", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(25.1), tangentialVelocityFromRPM(4000, flywheelDiameter))));
        NamedCommands.registerCommand("AdjustShooter4", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(109), tangentialVelocityFromRPM(4000, flywheelDiameter))));
        NamedCommands.registerCommand("AdjustShooter5", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(49), tangentialVelocityFromRPM(4000, flywheelDiameter))));
        NamedCommands.registerCommand("AdjustShooter6", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(28), tangentialVelocityFromRPM(4500, flywheelDiameter))));
        NamedCommands.registerCommand("AdjustShooter7", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(26.5), tangentialVelocityFromRPM(4500, flywheelDiameter))));
        NamedCommands.registerCommand("AdjustShooter8", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(110.75), tangentialVelocityFromRPM(3000, flywheelDiameter))));
    }

    private void initializeButtons(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, ButtonLayout layout) {
        switch (layout) {
            case PITCH_CHARACTERIZATION -> pitchCharacterizationLayout();
            case MAX_SPEEDS_CHARACTERIZATION -> maxSpeedsCharacterizationLayout(translationSup, strafeSup, rotationSup);
            case TELEOP -> teleopButtonsLayout(translationSup, strafeSup);
        }
    }

    private void teleopButtonsLayout(DoubleSupplier translationSup, DoubleSupplier strafeSup) {
        drXButton.whileTrue(new ShootToAmp(shooterSubsystem, swerve5990, leds));

        drAButton.whileTrue(shooterCommands.shootPhysics(13));
        drBButton.whileTrue(new ShootOnTheMove(shooterSubsystem, poseEstimator5990, shooterCommands, swerve5990, translationSup, strafeSup, 16));
        drYButton.whileTrue(new AlignWithAmp(swerve5990, translationSup, strafeSup));

        drLeftBumper.whileTrue(new AlignWithAmp(swerve5990, translationSup, strafeSup));

        drLeftTrigger.toggleOnFalse(shooterCommands.postIntake().withTimeout(0.65));
        drLeftTrigger.whileTrue((shooterCommands.floorIntake()));
        drRightTrigger.whileTrue(new IntakeCommands(intakeSubsystem).enableIntake(-0.9, true));

        drStartButton.onTrue(new InstantCommand(swerve5990::resetGyro));
        drBackButton.onTrue(new InstantCommand(swerve5990::lockSwerve));

        //Operator buttons:
        opAButton.whileTrue(shooterCommands.shootNote(SPEAKER_FRONT));
        opBButton.whileTrue(shooterCommands.shootNote(SPEAKER_BACK));
    }

    private void pitchCharacterizationLayout() {
        PitchCharacterization pitchCharacterization = new PitchCharacterization(shooterSubsystem);

        SysIdRoutine.Direction forward = SysIdRoutine.Direction.kForward;
        SysIdRoutine.Direction reverse = SysIdRoutine.Direction.kReverse;

        drAButton.whileTrue(pitchCharacterization.sysIdDynamicTest(forward));
        drBButton.whileTrue(pitchCharacterization.sysIdDynamicTest(reverse));
        drYButton.whileTrue(pitchCharacterization.sysIdQuastaticTest(forward));
        drXButton.whileTrue(pitchCharacterization.sysIdQuastaticTest(reverse));

        drLeftTrigger.whileTrue(shooterCommands.setPitchPosition(30));
        drRightTrigger.whileTrue(shooterCommands.setPitchPosition(90));
        drLeftBumper.whileTrue(shooterCommands.setPitchPosition(60));
        drRightBumper.whileTrue(shooterCommands.setPitchPosition(69));
    }

    private void maxSpeedsCharacterizationLayout(DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        drRightTrigger.whileTrue(
                new ParallelCommandGroup(
                        new MaxDrivetrainSpeedCharacterization(swerve5990, translationSup, strafeSup, rotationSup, () -> false),//, 5.1m/s
                        new MaxFlywheelSpeedCharacterization(shooterSubsystem) //5500 rpm flywheel
                )
        );
    }

    private enum ButtonLayout {
        PITCH_CHARACTERIZATION,
        MAX_SPEEDS_CHARACTERIZATION,
        TELEOP
    }
}
