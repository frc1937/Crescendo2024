// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AdjustShooter;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Mount;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterKick;
import frc.robot.commands.TeleopShooting;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TestMountCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
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
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final MountSubsystem mountSubsystem = new MountSubsystem();
    /* Commands */
    private final ShooterCommands shooterCommands = new ShooterCommands(shooterSubsystem, intakeSubsystem);
    private final TestMountCommand mountCommands = new TestMountCommand(mountSubsystem);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        swerveSubsystem,
                        () -> -driveController.getRawAxis(XboxController.Axis.kLeftY.value),
                        () -> -driveController.getRawAxis(XboxController.Axis.kLeftX.value),
                        () -> -driveController.getRawAxis(XboxController.Axis.kRightX.value),
                        drRightBumper
                )
        );

        NamedCommands.registerCommand("PrintTfilatHaDerech", Commands.print(TFILAT_HADERECH));
        NamedCommands.registerCommand("Intake", shooterCommands.intakeGet(false).withTimeout(3));
        NamedCommands.registerCommand("PostIntake", shooterCommands.postIntake());
        NamedCommands.registerCommand("ShooterKick", new ShooterKick(shooterSubsystem).withTimeout(SHOOTING_DELAY));
        NamedCommands.registerCommand("AdjustShooter1", new AdjustShooter(shooterSubsystem, -0.98));
        NamedCommands.registerCommand("AdjustShooter2", new AdjustShooter(shooterSubsystem, -0.85));
        NamedCommands.registerCommand("AdjustShooter3", new AdjustShooter(shooterSubsystem, 1.1));
        NamedCommands.registerCommand("AdjustShooter4", new AdjustShooter(shooterSubsystem, -0.99));

        autoChooser = AutoBuilder.buildAutoChooser("Crown (3)");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }


    private void configureBindings() {
        drStartButton.onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        DoubleSupplier translationSup = () -> -driveController.getRawAxis(XboxController.Axis.kLeftY.value);
        DoubleSupplier strafeSup = () -> -driveController.getRawAxis(XboxController.Axis.kLeftX.value);
        drAButton.whileTrue(
                new TeleopShooting(swerveSubsystem, shooterSubsystem, translationSup, strafeSup));

        drLeftBumper.whileTrue(shooterCommands.receiveFromFeeder());
        drRightTrigger.whileTrue(new IntakeCommand(intakeSubsystem, -0.9));
        drLeftTrigger.whileTrue(shooterCommands.intakeGet());

        drXButton.whileTrue(shooterCommands.shootNote(ShootingStates.AMP));
        drBButton.whileTrue(new Mount(mountSubsystem));

        //Operator buttons:
        opAButton.whileTrue(shooterCommands.shootNote(ShootingStates.SPEAKER_FRONT));
        opBButton.whileTrue(shooterCommands.shootNote(ShootingStates.SPEAKER_BACK));
        opXButton.whileTrue(shooterCommands.shootNote(ShootingStates.STAGE_FRONT));
        opYButton.whileTrue(shooterCommands.shootNote(ShootingStates.AMP));

        opRightBumper.whileTrue(shooterCommands.accelerateFlywheel(ShootingStates.STAGE_FRONT));
        opRightBumper.toggleOnFalse(shooterCommands.stopShooter());

        opLeftBumper.whileTrue(shooterCommands.accelerateFlywheel(ShootingStates.SPEAKER_FRONT));
//        opLeftBumper.toggleOnFalse(shooterCommands.stopShooter());
        opStartButton.whileTrue(mountCommands.testMountCommand());
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void infrequentPeriodic() {
        swerveSubsystem.infrequentPeriodic();
    }
}
