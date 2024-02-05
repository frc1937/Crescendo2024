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
import frc.robot.commands.TeleopSwerve;
import frc.robot.photonvision.ChaseTag;
import frc.robot.photonvision.PoseEstimator;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;

public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final SendableChooser<Command> autoChooser;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton intakeStartButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton intakeStopButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton chaseTagCommand = new JoystickButton(driver, XboxController.Button.kX.value);
    /* Subsystems */
    private final SwerveSubsystem swerve = new SwerveSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();

    /* Photonvision */
    private final PhotonCamera photonCamera = new PhotonCamera("Photon1937");
    private final PoseEstimator poseEstimator = new PoseEstimator(photonCamera, swerve);

    private final ChaseTag chaseTag = new ChaseTag(photonCamera, swerve, poseEstimator::getCurrentPose);

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
        zeroGyro.onTrue(new InstantCommand(swerve::zeroGyro));

        intakeStartButton.onTrue(new InstantCommand(() -> {
            intake.setSpeedPercentage(0.8);
        }));

        intakeStopButton.onTrue(new InstantCommand(intake::stopMotor));

        chaseTagCommand.whileTrue(chaseTag);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * A custom periodic function that should be call infrequenty compared to the regular periodic
     */
    public void infrequentPeriodic() {
        swerve.infrequentPeriodic();
    }
}
