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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AdjustShooter;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MountCommand;
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
//    private AddressableLED m_led;
//    private AddressableLEDBuffer m_ledBuffer;
//    // Store what the last hue of the first pixel is
//    private int m_rainbowFirstPixelHue;
    private final XboxController driveController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    private final SendableChooser<Command> autoChooser;

    /* MAIN-DRIVER */
    private final JoystickButton startButton = new JoystickButton(driveController, XboxController.Button.kStart.value);
    private final TriggerButton leftTrigger = new TriggerButton(driveController, XboxController.Axis.kLeftTrigger);
    private final TriggerButton rightTrigger = new TriggerButton(driveController, XboxController.Axis.kRightTrigger);
    private final JoystickButton aButton = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(driveController, XboxController.Button.kB.value);
    private final JoystickButton leftBumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rightBumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);
    private final JoystickButton backButton = new JoystickButton(driveController, XboxController.Button.kBack.value);
    /* OPERATOR */
    private final TriggerButton accelerateFlywheelButton = new TriggerButton(operatorController, XboxController.Axis.kRightTrigger);
    private final JoystickButton randomPitchYButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
    private final JoystickButton opAButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
    private final JoystickButton opBButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton opYButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
    private final JoystickButton opXButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
    private final JoystickButton opRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
    private final JoystickButton opLeftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton opStartButton = new JoystickButton(operatorController, XboxController.Button.kStart.value);
    /* Subsystems */
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
                        rightBumper
                )
        );

        NamedCommands.registerCommand("Shifra2", shooterCommands.intakeGet().withTimeout(5));
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
        backButton.whileTrue(Navigate.navigateToAmplifier());

        startButton.onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        leftTrigger.whileTrue(shooterCommands.intakeGet().andThen(shooterCommands.setKickerSpeed(-0.8).withTimeout(0.7)));

        aButton.whileTrue(
                new TeleopShooting(swerveSubsystem, shooterSubsystem,
                        () -> -driveController.getRawAxis(XboxController.Axis.kLeftY.value),
                        () -> -driveController.getRawAxis(XboxController.Axis.kLeftX.value))
        );

        bButton.whileTrue(new MountCommand(mountSubsystem));

        leftBumper.whileTrue(shooterCommands.receiveFromFeeder());

        rightTrigger.whileTrue(new IntakeCommand(intakeSubsystem, -0.9));

        //sagi:
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
//        m_led = new AddressableLED(0);
//        m_ledBuffer = new AddressableLEDBuffer(160);
//        m_led.setLength(m_ledBuffer.getLength());
//
//        // Set the data
//        m_led.setData(m_ledBuffer);
//        m_led.start();


//        AddressableLED led = new AddressableLED(0);
//        AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(2000);
//
//        led.stop();
//
//        for (int i = 0; i < ledBuffer.getLength(); i++) {
//            ledBuffer.setRGB(i, 0, 0, 122);
//        }
//
//        led.setLength(ledBuffer.getLength());
//        led.setData(ledBuffer);
//
//        led.start();
    }

    public void robotPeriodic() {
//        rainbow();
        // Set the LEDs
//        m_led.setData(m_ledBuffer);
    }

//    private void rainbow() {
//        // For every pixel
//        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
//            // Calculate the hue - hue is easier for rainbows because the color
//            // shape is a circle so only one value needs to precess
//            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
//            // Set the value
//            m_ledBuffer.setHSV(i, hue, 255, 128);
//        }
//        // Increase by to make the rainbow "move"
//        m_rainbowFirstPixelHue += 3;
//        // Check bounds
//        m_rainbowFirstPixelHue %= 180;
//    }
}
