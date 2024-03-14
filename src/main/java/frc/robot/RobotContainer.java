// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.ShootingStates;
import frc.robot.util.TriggerButton;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;
import static frc.robot.Constants.TFILAT_HADERECH;

public class RobotContainer {
    private final XboxController driveController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    private final SendableChooser<Command> autoChooser;

    /* MAIN-DRIVER */
    private final JoystickButton drAButton = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton drBButton = new JoystickButton(driveController, XboxController.Button.kB.value);
    private final JoystickButton driveYButton = new JoystickButton(driveController, XboxController.Button.kY.value);
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
    private final AutonomousShooter autonomousShooter = new AutonomousShooter(shooterSubsystem);

    public RobotContainer() {
        NamedCommands.registerCommand("PrintTfilatHaDerech", Commands.print(TFILAT_HADERECH));

        NamedCommands.registerCommand("Intake", shooterCommands.intakeGet(false).withTimeout(3));
        NamedCommands.registerCommand("PostIntake", shooterCommands.postIntake());
        NamedCommands.registerCommand("ShooterKick", new ShooterKick(shooterSubsystem).withTimeout(SHOOTING_DELAY));

        NamedCommands.registerCommand("AdjustShooter1", autonomousShooter.adjustShooter(136, 3500));
        NamedCommands.registerCommand("AdjustShooter2", autonomousShooter.adjustShooter(134.4, 4000));
        NamedCommands.registerCommand("AdjustShooter3", autonomousShooter.adjustShooter(64, 4000));
        NamedCommands.registerCommand("AdjustShooter4", autonomousShooter.adjustShooter(125, 4000));

        autoChooser = AutoBuilder.buildAutoChooser("Crown (3)");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }


    private void configureBindings() {
        DoubleSupplier translationSup = () -> -driveController.getRawAxis(XboxController.Axis.kLeftY.value);
        DoubleSupplier strafeSup = () -> -driveController.getRawAxis(XboxController.Axis.kLeftX.value);
        DoubleSupplier rotationSup = () -> -driveController.getRawAxis(XboxController.Axis.kRightX.value);

        drivetrain.setDefaultCommand(
                new TeleopSwerve(
                        drivetrain,
                        translationSup,
                        strafeSup,
                        rotationSup,
                        drRightBumper
                )
        );

        drAButton.whileTrue(new TeleopShooting(drivetrain, shooterSubsystem, translationSup, strafeSup));

        drLeftTrigger.whileTrue((shooterCommands.intakeGet()));
        drLeftBumper.whileTrue(shooterCommands.receiveFromFeeder());
        drRightTrigger.whileTrue(new IntakeCommand(intakeSubsystem, -0.9));

        drStartButton.onTrue(new InstantCommand(drivetrain::zeroGyro));
        drXButton.whileTrue(shooterCommands.shootNote(ShootingStates.SPEAKER_FRONT));

        //Operator buttons:
        opAButton.whileTrue(shooterCommands.shootNote(ShootingStates.SPEAKER_FRONT));
        opBButton.whileTrue(shooterCommands.shootNote(ShootingStates.SPEAKER_BACK));
        opYButton.whileTrue(shooterCommands.shootToAmp(ShootingStates.AMP));
        opXButton.whileTrue(shooterCommands.shootNote(ShootingStates.STAGE_FRONT));

        mountSubsystem.setDefaultCommand(
                mountCommands.startManualMount(
                        () -> -operatorController.getRawAxis(XboxController.Axis.kLeftY.value),
                        () -> -operatorController.getRawAxis(XboxController.Axis.kRightY.value)
                )
        );

        opStartButton.whileTrue(mountCommands.startAutomaticMount());
    }


    public Command getAutonomousCommand() {
        return new InstantCommand(
                () -> shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(90)),
                shooterSubsystem)
            .andThen(new WaitCommand(3))
            .andThen(new InstantCommand(
                () -> shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(0)),
                shooterSubsystem))
            .andThen(new WaitCommand(3))
            .andThen(new InstantCommand(
                () -> shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(45)),
                shooterSubsystem))
            .andThen(new WaitCommand(3))
            .andThen(new InstantCommand(
                () -> shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(0)),
                shooterSubsystem))
            .andThen(new InstantCommand(
                () -> shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(30)),
                shooterSubsystem))
            .andThen(new WaitCommand(3))
            .andThen(new InstantCommand(
                () -> shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(60)),
                shooterSubsystem))
            .andThen(new WaitCommand(3))
            .andThen(new InstantCommand(
                () -> shooterSubsystem.setPitchGoal(Rotation2d.fromDegrees(0)),
                shooterSubsystem));
        // return autoChooser.getSelected();
    }

    public void infrequentPeriodic() {
        drivetrain.infrequentPeriodic();
    }
}
