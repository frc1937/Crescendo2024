// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.VisionDrive;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final SendableChooser<Command> autoChooser;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyroButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Vision Vision = new Vision();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    public RobotContainer() {
        JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        robotCentric
                )
        );

        autoChooser = AutoBuilder.buildAutoChooser("HoopTest");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }


    private void configureBindings() {
        zeroGyroButton.onTrue(new InstantCommand(swerve::zeroGyro));
        xButton.whileTrue(new SpinIntake(intake, 0));
    }


    public Command getAutonomousCommand() {
        VisionDrive autonomousCommand = new VisionDrive(Vision, swerve);
        return autonomousCommand;
    }
}
