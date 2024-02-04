// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionPoseEstimator;

import static frc.robot.Constants.DriveConstants.ROTATION_AXIS;
import static frc.robot.Constants.DriveConstants.STRAFE_AXIS;
import static frc.robot.Constants.DriveConstants.TRANSLATION_AXIS;
import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;

public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final SendableChooser<Command> autoChooser;
    /* Drive Buttons */
    private final JoystickButton zeroGyroButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton chaseTagButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton shooterButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton rotatePivot = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final VisionPoseEstimator visionPoseEstimator = new VisionPoseEstimator();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    /* Commands */
    private final ShooterCommands shooterCommands = new ShooterCommands(shooterSubsystem);
    private final ChaseTagCommand chaseTagCommand = new ChaseTagCommand(swerveSubsystem, visionPoseEstimator);

    public RobotContainer() {
        JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

        swerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        swerveSubsystem,
                        () -> -driver.getRawAxis(TRANSLATION_AXIS),
                        () -> -driver.getRawAxis(STRAFE_AXIS),
                        () -> -driver.getRawAxis(ROTATION_AXIS),
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
        intakeButton.whileTrue(new IntakeCommand(intakeSubsystem, INTAKE_SPEED));

        /* THESE ARE ARBITRARY VALUES FOR PID TESTING, WILL FINALIZE */
        Rotation2d rotation = Rotation2d.fromDegrees(50);

        shooterButton.whileTrue(shooterCommands.rotateFlywheels(0.5));
        rotatePivot.whileTrue(shooterCommands.rotatePivot(rotation));
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
