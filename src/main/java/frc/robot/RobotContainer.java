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
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MountCommand;
import frc.robot.commands.MountCommands;
import frc.robot.commands.PrepareShooter;
import frc.robot.commands.ShootToAmp;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterKick;
import frc.robot.commands.TeleOpDrive;
import frc.robot.commands.TeleOpShoot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.TriggerButton;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.ASSIST_TARGET;
import static frc.robot.Constants.ShootingConstants.SPEAKER_TARGET;
import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
import static frc.robot.Constants.ShootingConstants.SPEAKER_BACK;
import static frc.robot.Constants.ShootingConstants.SPEAKER_FRONT;

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
    /* Commands */
    private final MountCommands mountCommands = new MountCommands(mountSubsystem);
    private final ShooterCommands shooterCommands = new ShooterCommands(shooterSubsystem, intakeSubsystem);

    public RobotContainer() {
//        NamedCommands.registerCommand("PrintTfilatHaDerech", Commands.print(TFILAT_HADERECH));

        NamedCommands.registerCommand("Intake", shooterCommands.floorIntake().withTimeout(2));
        NamedCommands.registerCommand("PostIntake", shooterCommands.postIntake());
        NamedCommands.registerCommand("Rotate", new AimAtSpeaker(drivetrain, 1));
        NamedCommands.registerCommand("Half-Rotate", new AimAtSpeaker(drivetrain, 0.5));

        NamedCommands.registerCommand("IntakeUnicorn", shooterCommands.floorIntake().withTimeout(2.7));

        NamedCommands.registerCommand("ShooterKick", new ShooterKick(shooterSubsystem).withTimeout(SHOOTING_DELAY));

        NamedCommands.registerCommand("AdjustShooter1", new PrepareShooter(shooterSubsystem,
                new ShooterSubsystem.Reference(Rotation2d.fromDegrees(108), RPM.of(3000))));
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

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }


    private void configureBindings() {
        DoubleSupplier translationSup = () -> -driveController.getRawAxis(XboxController.Axis.kLeftY.value);
        DoubleSupplier strafeSup = () -> -driveController.getRawAxis(XboxController.Axis.kLeftX.value);
        DoubleSupplier rotationSup = () -> MathUtil.applyDeadband(
                -driveController.getRawAxis(XboxController.Axis.kRightX.value), Constants.STICK_DEADBAND);

        drivetrain.setDefaultCommand(
                new TeleOpDrive(
                        drivetrain,
                        translationSup,
                        strafeSup,
                        rotationSup,
                        drRightBumper
                )
        );

        drAButton.whileTrue(new TeleOpShoot(drivetrain, shooterSubsystem, SPEAKER_TARGET, translationSup, strafeSup, false));
        drBButton.whileTrue(new TeleOpShoot(drivetrain, shooterSubsystem, SPEAKER_TARGET, translationSup, strafeSup, true));
        drYButton.whileTrue(new TeleOpShoot(drivetrain, shooterSubsystem, ASSIST_TARGET, translationSup, strafeSup, true));


//        drLeftBumper.whileTrue(shooterCommands.receiveFromFeeder());
        drLeftTrigger.toggleOnFalse(shooterCommands.postIntake().withTimeout(0.65));
        drLeftTrigger.whileTrue((shooterCommands.floorIntake()));

        drRightTrigger.whileTrue(new IntakeCommand(intakeSubsystem, -0.9));

        drStartButton.onTrue(new InstantCommand(drivetrain::zeroGyro));

        drXButton.whileTrue(new ShootToAmp(shooterSubsystem, drivetrain));
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

        opStartButton.whileTrue(mountCommands.startAutomaticMount());
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
