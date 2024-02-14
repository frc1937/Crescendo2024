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
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.TriggerButton;

import static frc.robot.Constants.IntakeConstants.INTAKE_SPEED;

public class RobotContainer {
    private final XboxController driver = new XboxController(0);
    private final SendableChooser<Command> autoChooser;

    /* Drive Buttons */
    private final TriggerButton zeroGyroButton = new TriggerButton(driver, XboxController.Axis.kRightTrigger);
    private final TriggerButton intakeButton = new TriggerButton(driver, XboxController.Axis.kLeftTrigger);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);

    /* Subsystems */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    /* Commands */
    private final ShooterCommands shooterCommands = new ShooterCommands(shooterSubsystem, intakeSubsystem);

    public RobotContainer() {
        JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

        swerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        swerveSubsystem,
                        () -> -driver.getRawAxis(XboxController.Axis.kLeftY.value),
                        () -> -driver.getRawAxis(XboxController.Axis.kLeftX.value),
                        () -> -driver.getRawAxis(XboxController.Axis.kRightX.value),
                        robotCentric
                )
        );

        NamedCommands.registerCommand("pickupIntake", new IntakeCommand(intakeSubsystem, INTAKE_SPEED));

        autoChooser = AutoBuilder.buildAutoChooser("HoopTest");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }


    private void configureBindings() {
//        zeroGyroButton.onTrue(new InstantCommand(swerveSubsystem::zeroGyro));
        zeroGyroButton.onTrue(new InstantCommand(swerveSubsystem::resetPose));

        intakeButton.onTrue(shooterCommands.intakeGet()
                .andThen(shooterCommands.setKickerSpeed(-0.8).withTimeout(0.7)));

        yButton.whileTrue(shooterCommands.receiveFromFeeder().andThen(shooterCommands.setKickerSpeed(-0.8)
                .withTimeout(0.7)));

        bButton.whileTrue(shooterCommands.shootNote(125, 0.08));

        xButton.whileTrue(shooterCommands.shootNote(80, 0.9));

//        aButton.whileTrue(shooterCommands.setAngle(60));
//
//        aButton.whileTrue(new TeleopShooting(swerveSubsystem, shooterSubsystem,
//                () -> -driver.getRawAxis(XboxController.Axis.kLeftY.value),
//                () -> -driver.getRawAxis(XboxController.Axis.kLeftX.value)));

        // xButton.whileTrue(shooterCommands.setAngle(120));
//todo reverse intake
        // AMP shoot
    }

    //todo:
    // quasistatic table of values
    // max acc, max speed, (Ask CAD people), all these goofy constants
    //

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void infrequentPeriodic() {
        swerveSubsystem.infrequentPeriodic();
    }
}
