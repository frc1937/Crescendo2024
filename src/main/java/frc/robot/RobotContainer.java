package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.leds.ColourByShooter;
import frc.robot.subsystems.*;
import frc.robot.util.Camera;
import frc.robot.util.Controller;
import frc.robot.vision.VisionPoseEstimator;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShootingConstants.*;
import static frc.robot.Constants.Transforms.FRONT_CAMERA_TO_ROBOT;
import static frc.robot.util.Controller.Inputs.*;

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
    private final Trigger drLeftTrigger = driveController.getStick(LEFT_STICK);
    private final Trigger drRightTrigger = driveController.getStick(RIGHT_STICK);
    private final Trigger drLeftBumper = driveController.getButton(LEFT_BUMPER);
    private final Trigger drRightBumper = driveController.getButton(RIGHT_BUMPER);

    /* OPERATOR */
    private final Trigger opAButton = operatorController.getButton(A);
    private final Trigger opBButton = operatorController.getButton(B);
    private final Trigger opXButton = operatorController.getButton(X);

    /* Subsystems */
    private final VisionPoseEstimator visionPoseEstimator;
    private final DrivetrainSubsystem drivetrain;
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final MountSubsystem mountSubsystem = new MountSubsystem();
    private final LEDsSubsystem leds = new LEDsSubsystem();
    /* Commands */
    private final ShooterCommands shooterCommands = new ShooterCommands(shooterSubsystem, intakeSubsystem, leds);

    public RobotContainer() {
        visionPoseEstimator = new VisionPoseEstimator(
                new Camera("Front1937", FRONT_CAMERA_TO_ROBOT)
        );

        drivetrain = new DrivetrainSubsystem(visionPoseEstimator);

        NamedCommands.registerCommand("Intake", shooterCommands.floorIntake().withTimeout(2));
        NamedCommands.registerCommand("PostIntake", shooterCommands.postIntake());
        NamedCommands.registerCommand("Rotate", new AimAtSpeaker(drivetrain, 1));
        NamedCommands.registerCommand("Half-Rotate", new AimAtSpeaker(drivetrain, 0.5));

        NamedCommands.registerCommand("IntakeUnicorn", shooterCommands.floorIntake().withTimeout(2.7));

        NamedCommands.registerCommand("ShooterKick", new ShooterKick(shooterSubsystem).withTimeout(SHOOTING_DELAY));

        NamedCommands.registerCommand("AdjustShooter1", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(108.25), RPM.of(3000))));
        NamedCommands.registerCommand("AdjustShooter2", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(114), RPM.of(3000))));
        NamedCommands.registerCommand("AdjustShooter3", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(25.1), RPM.of(4000))));
        NamedCommands.registerCommand("AdjustShooter4", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(109), RPM.of(4000))));
        NamedCommands.registerCommand("AdjustShooter5", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(49), RPM.of(4000))));
        NamedCommands.registerCommand("AdjustShooter6", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(28), RPM.of(4500))));
        NamedCommands.registerCommand("AdjustShooter7", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(26.5), RPM.of(4500))));
        NamedCommands.registerCommand("AdjustShooter8", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(110.75), RPM.of(3000))));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }


    private void configureBindings() {
        DoubleSupplier translationSup = () -> -driveController.getRawAxis(LEFT_Y);
        DoubleSupplier strafeSup = () -> -driveController.getRawAxis(LEFT_X);
        DoubleSupplier rotationSup = () -> MathUtil.applyDeadband(-driveController.getRawAxis(RIGHT_X), Constants.STICK_DEADBAND);

        drivetrain.setDefaultCommand(
                new TeleOpDrive(
                        drivetrain,
                        translationSup,
                        strafeSup,
                        rotationSup,
                        () -> false
                )
        );

        leds.setDefaultCommand(new ColourByShooter(leds, shooterSubsystem));

        drAButton.whileTrue(new TeleOpShoot(drivetrain, shooterSubsystem, leds, SPEAKER_TARGET, translationSup, strafeSup, false, Seconds.of(2.d)));
        drBButton.whileTrue(new TeleOpShoot(drivetrain, shooterSubsystem, leds, SPEAKER_TARGET, translationSup, strafeSup, true, Seconds.of(5)));
        // drYButton.whileTrue(new TeleOpShoot(drivetrain, shooterSubsystem, leds, ASSIST_TARGET, translationSup, strafeSup, true, Seconds.of(1.5)));
        drYButton.whileTrue(shooterCommands.shootNote(ASSIST));
        drXButton.whileTrue(new ShootToAmp(shooterSubsystem, drivetrain, leds));

        drLeftBumper.whileTrue(new AlignWithAmp(drivetrain, translationSup, strafeSup));
        drLeftTrigger.toggleOnFalse(shooterCommands.postIntake().withTimeout(0.65));
        drLeftTrigger.whileTrue((shooterCommands.floorIntake()));
        drRightBumper.whileTrue(new AlignWithChain(drivetrain, rotationSup, rotationSup));
        drRightTrigger.whileTrue(new IntakeCommand(intakeSubsystem, -0.9));

        drStartButton.onTrue(new InstantCommand(drivetrain::zeroGyro));

        //Operator buttons:
        opAButton.whileTrue(shooterCommands.shootNote(SPEAKER_FRONT));
        opBButton.whileTrue(shooterCommands.shootNote(SPEAKER_BACK));
        opXButton.whileTrue(shooterCommands.shootNote(new ShooterSubsystem.Reference(Rotation2d.fromDegrees(100), RPM.of(-1000))));

        mountSubsystem.setDefaultCommand(
                new MountCommand(mountSubsystem,
                        () -> MathUtil.applyDeadband(-operatorController.getRawAxis(LEFT_Y), Constants.STICK_DEADBAND*0.5),
                        () -> MathUtil.applyDeadband(-operatorController.getRawAxis(RIGHT_Y), Constants.STICK_DEADBAND*0.5)
                )
        );
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void infrequentPeriodic() {
        drivetrain.infrequentPeriodic();
    }

    public void robotPeriodic() {

    }
}
