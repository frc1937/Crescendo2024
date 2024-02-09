// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;

public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final SendableChooser<Command> autoChooser;

    /* Drive Buttons */
    private final JoystickButton zeroGyroButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Subsystems */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    /* Commands */
    private final ShooterCommands shooterCommands = new ShooterCommands(shooterSubsystem, intakeSubsystem);

    public RobotContainer() {
        JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
//TODO: Commented out so robot doesn't yeet itself

//        swerveSubsystem.setDefaultCommand(
//                new TeleopSwerve(
//                        swerveSubsystem,
//                        () -> -driver.getRawAxis(XboxController.Axis.kLeftY.value),
//                        () -> -driver.getRawAxis(XboxController.Axis.kLeftX.value),
//                        () -> -driver.getRawAxis(XboxController.Axis.kRightX.value),
//                        robotCentric
//                )
//        );

        NamedCommands.registerCommand("pickupIntake", new IntakeCommand(intakeSubsystem, INTAKE_SPEED));

        autoChooser = AutoBuilder.buildAutoChooser("HoopTest");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }


    private void configureBindings() {
        zeroGyroButton.onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        aButton.whileTrue(shooterCommands.manualFloorIntake());
        bButton.whileTrue(shooterCommands.restPivotHighAndShootNote());
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
