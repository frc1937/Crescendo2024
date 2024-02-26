// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AdjustShooter;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Navigate;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterKick;
import frc.robot.commands.TeleopShooting;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MountSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ShootingStates;
import frc.robot.util.TriggerButton;

import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;

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
    /* Subsystems */
//    private final LedsSubsystem ledsSubsystem = new LedsSubsystem();
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final MountSubsystem mountSubsystem = new MountSubsystem();
    /* Commands */
    private final ShooterCommands shooterCommands = new ShooterCommands(shooterSubsystem, intakeSubsystem);

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

        NamedCommands.registerCommand("Intake", shooterCommands.intakeGet().withTimeout(5));
        NamedCommands.registerCommand("AdjustShooter22", new AdjustShooter(shooterSubsystem, 0.8));
        NamedCommands.registerCommand("AdjustShooter23", new AdjustShooter(shooterSubsystem, 0.8));
        NamedCommands.registerCommand("AdjustShooter1", new AdjustShooter(shooterSubsystem, 1.15));
        NamedCommands.registerCommand("AdjustShooter2", new AdjustShooter(shooterSubsystem, 0.9));
        NamedCommands.registerCommand("AdjustShooter3", new AdjustShooter(shooterSubsystem, 0.8));
        NamedCommands.registerCommand("AdjustShooter4", new AdjustShooter(shooterSubsystem, 0.9));
        NamedCommands.registerCommand("ShooterKick", new ShooterKick(shooterSubsystem).withTimeout(SHOOTING_DELAY));

        autoChooser = AutoBuilder.buildAutoChooser("Eyal");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }


    private void configureBindings() {
        drXButton.whileTrue(shooterCommands.setAngle(0));
        driveYButton.whileTrue(shooterCommands.setAngle(70));
        drBButton.whileTrue(shooterCommands.setAngle(81));

        drBackButton.whileTrue(Navigate.navigateToAmplifier());
        drStartButton.onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        drAButton.whileTrue(
                new TeleopShooting(swerveSubsystem, shooterSubsystem,
                        () -> -driveController.getRawAxis(XboxController.Axis.kLeftY.value),
                        () -> -driveController.getRawAxis(XboxController.Axis.kLeftX.value))
        );

//        drBButton.whileTrue(new MountCommand(mountSubsystem));

        drLeftBumper.whileTrue(shooterCommands.receiveFromFeeder());
        drRightTrigger.whileTrue(new IntakeCommand(intakeSubsystem, -0.9));
        drLeftTrigger.whileTrue(shooterCommands.intakeGet());

        //Operator buttons:
        opAButton.whileTrue(shooterCommands.shootNote(ShootingStates.SPEAKER_FRONT));
        opBButton.whileTrue(shooterCommands.shootNote(ShootingStates.SPEAKER_BACK));
        opYButton.whileTrue(shooterCommands.shootNote(ShootingStates.AMP));
        opXButton.whileTrue(shooterCommands.shootNote(ShootingStates.STAGE_FRONT));

        opRightBumper.whileTrue(shooterCommands.accelerateFlywheel(ShootingStates.STAGE_FRONT));
        opRightBumper.toggleOnFalse(shooterCommands.stopShooter());

        opLeftBumper.whileTrue(shooterCommands.accelerateFlywheel(ShootingStates.SPEAKER_FRONT));
        opLeftBumper.toggleOnFalse(shooterCommands.stopShooter());
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void infrequentPeriodic() {
        swerveSubsystem.infrequentPeriodic();
    }

    public void robotInit() {
        AddressableLED led = new AddressableLED(0);
        AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(105);

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 122);
        }

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);

        led.start();
    }
}
