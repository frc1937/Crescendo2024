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
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionPoseEstimator;

import java.util.function.BooleanSupplier;

public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final SendableChooser<Command> autoChooser;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyroButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton chaseTagButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kX.value);
    /* Subsystems */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator();
    private BooleanSupplier isIntakeInverted;
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(isIntakeInverted);
    private final IntakeCommands intakeCommands = new IntakeCommands(intakeSubsystem);
    /* PhotonVision */
    private final ChaseTagCommand chaseTagCommand = new ChaseTagCommand(swerveSubsystem, visionPoseEstimator);

    public RobotContainer() {
        JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

        swerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        swerveSubsystem,
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
        zeroGyroButton.onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
        chaseTagButton.whileTrue(chaseTagCommand);
        intakeButton.whileTrue(intakeCommands.startIntake(0.8));
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
