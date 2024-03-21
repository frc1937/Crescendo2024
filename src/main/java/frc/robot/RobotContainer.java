// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimAtSpeaker;
import frc.robot.commands.AlignWithAmp;
import frc.robot.commands.AlignWithChain;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MountCommand;
import frc.robot.commands.PrepareShooter;
import frc.robot.commands.ShootToAmp;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterKick;
import frc.robot.commands.TeleOpDrive;
import frc.robot.commands.TeleOpShoot;
import frc.robot.commands.leds.ColourByShooter;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.MountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.TriggerButton;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.ShootingConstants.ASSIST_TARGET;
import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.SPEAKER_BACK;
import static frc.robot.Constants.ShootingConstants.SPEAKER_FRONT;
import static frc.robot.Constants.ShootingConstants.SPEAKER_TARGET;

public class RobotContainer {
    private final XboxController driveController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    private final SendableChooser<Command> autoChooser;

    /* MAIN-DRIVER */
    private final JoystickButton drAButton = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton drYButton = new JoystickButton(driveController, XboxController.Button.kY.value);
    private final JoystickButton drBButton = new JoystickButton(driveController, XboxController.Button.kB.value);
    private final JoystickButton drXButton = new JoystickButton(driveController, XboxController.Button.kX.value);
    private final JoystickButton drStartButton = new JoystickButton(driveController, XboxController.Button.kStart.value);
    private final TriggerButton drLeftTrigger = new TriggerButton(driveController, XboxController.Axis.kLeftTrigger);
    private final TriggerButton drRightTrigger = new TriggerButton(driveController, XboxController.Axis.kRightTrigger);
    private final JoystickButton drLeftBumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton drRightBumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
    private final JoystickButton drBackButton = new JoystickButton(driveController, XboxController.Button.kBack.value);
    /* OPERATOR */
    private final JoystickButton opAButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
    private final JoystickButton opBButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton opYButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
    private final JoystickButton opXButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
    private final JoystickButton opRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    private final JoystickButton opLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton opStartButton = new JoystickButton(operatorController, XboxController.Button.kStart.value);
    private final TriggerButton opLeftTrigger = new TriggerButton(operatorController, XboxController.Axis.kLeftTrigger);
    /* Subsystems */
    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final MountSubsystem mountSubsystem = new MountSubsystem();
    private final LEDsSubsystem leds = new LEDsSubsystem();
    /* Commands */
    private final ShooterCommands shooterCommands = new ShooterCommands(shooterSubsystem, intakeSubsystem, leds);

    public RobotContainer() {
//        NamedCommands.registerCommand("PrintTfilatHaDerech", Commands.print(TFILAT_HADERECH));

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
        DoubleSupplier translationSup = () -> -driveController.getRawAxis(XboxController.Axis.kLeftY.value);
        DoubleSupplier strafeSup = () -> -driveController.getRawAxis(XboxController.Axis.kLeftX.value);
        DoubleSupplier rotationSup = () -> MathUtil.applyDeadband(-driveController.getRawAxis(XboxController.Axis.kRightX.value), Constants.STICK_DEADBAND);

        drivetrain.setDefaultCommand(
                new TeleOpDrive(
                        drivetrain,
                        translationSup,
                        strafeSup,
                        rotationSup,
                        () -> false
                )
        );
//op: feeder, me
        leds.setDefaultCommand(new ColourByShooter(leds, shooterSubsystem));

        drAButton.whileTrue(new TeleOpShoot(drivetrain, shooterSubsystem, leds, SPEAKER_TARGET, translationSup, strafeSup, false, Seconds.of(3.7)));
        drBButton.whileTrue(new TeleOpShoot(drivetrain, shooterSubsystem, leds, SPEAKER_TARGET, translationSup, strafeSup, true, Seconds.of(5)));
        drYButton.whileTrue(new TeleOpShoot(drivetrain, shooterSubsystem, leds, ASSIST_TARGET, translationSup, strafeSup, true, Seconds.of(1.85)));
        drXButton.whileTrue(new ShootToAmp(shooterSubsystem, drivetrain, leds));

        drLeftBumper.whileTrue(new AlignWithAmp(drivetrain, translationSup, strafeSup));
        drLeftTrigger.toggleOnFalse(shooterCommands.postIntake().withTimeout(0.65));
        drLeftTrigger.whileTrue((shooterCommands.floorIntake()));
        drRightBumper.whileTrue(new AlignWithChain(drivetrain, rotationSup, rotationSup));
        drRightTrigger.whileTrue(new IntakeCommand(intakeSubsystem, -0.9));

        drStartButton.onTrue(new InstantCommand(drivetrain::zeroGyro));
        //        drXButton.whileTrue(shooterCommands.shootNote(SPEAKER_FRONT));
//        driveYButton.whileTrue(shooterCommands.shootNote(SPEAKER_BACK));

        //Operator buttons:
        opAButton.whileTrue(shooterCommands.shootNote(SPEAKER_FRONT));
        opBButton.whileTrue(shooterCommands.shootNote(SPEAKER_BACK));
        opXButton.whileTrue(shooterCommands.shootNote(new ShooterSubsystem.Reference(Rotation2d.fromDegrees(100), RPM.of(-1000))));

        mountSubsystem.setDefaultCommand(
                new MountCommand(mountSubsystem,
                        () -> MathUtil.applyDeadband(-operatorController.getRawAxis(XboxController.Axis.kLeftY.value), Constants.STICK_DEADBAND*0.5),
                        () -> MathUtil.applyDeadband(-operatorController.getRawAxis(XboxController.Axis.kRightY.value), Constants.STICK_DEADBAND*0.5)
                )
        );
    }


    public Command getAutonomousCommand() {
//        return new SequentialCommandGroup(
//                autonomousShooter.adjustShooter(112, 2000),
//                new ShooterKick(shooterSubsystem).withTimeout(SHOOTING_DELAY),
//                //shoot first note
//
//                new ParallelCommandGroup(
//                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Base to NOTE 1 (3)")),
//                        shooterCommands.floorIntake(true).withTimeout(2.7)
//                ),
//                new ParallelCommandGroup(
//                        new RotateToSpeaker(drivetrain).withTimeout(1).andThen(new RotateToSpeaker(drivetrain)),
//                        autonomousShooter.adjustShooter(30, 3000)
//                ),
//
//                new ShooterKick(shooterSubsystem).withTimeout(SHOOTING_DELAY),
//                //SHOOT NOTE 1 ^
//
//                new ParallelCommandGroup(
//                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("NOTE 1 to NOTE 2 (0.3)")),
//                        shooterCommands.floorIntake(true).withTimeout(3)
//                ).andThen(new ParallelCommandGroup(
//                        new RotateToSpeaker(drivetrain).withTimeout(2.4),
//                        autonomousShooter.adjustShooter(34, 3000)
//                )),
//
//                new ShooterKick(shooterSubsystem).withTimeout(SHOOTING_DELAY),
//                //SHOOT note 2 ^
//
//                new ParallelCommandGroup(
//                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("NOTE 2 to NOTE 3 (0.3)")),
//                        shooterCommands.floorIntake(true).withTimeout(3)
//                ).andThen(new ParallelCommandGroup(
//                        new RotateToSpeaker(drivetrain).withTimeout(2.4),
//                        autonomousShooter.adjustShooter(30, 3000)
//                )),
//
//                new WaitCommand(0.5),
//                new ShooterKick(shooterSubsystem).withTimeout(SHOOTING_DELAY)
//        );
        return autoChooser.getSelected();
    }

    public void infrequentPeriodic() {
        drivetrain.infrequentPeriodic();
    }

    public void robotPeriodic() {

        SmartDashboard.putNumber("op/nice1",
                MathUtil.applyDeadband(-operatorController.getRawAxis(XboxController.Axis.kLeftY.value), Constants.STICK_DEADBAND*0.5));
        SmartDashboard.putNumber("op/nice2",
                MathUtil.applyDeadband(-operatorController.getRawAxis(XboxController.Axis.kRightY.value), Constants.STICK_DEADBAND*0.5)
        );


    }
}
