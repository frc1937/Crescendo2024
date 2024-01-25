// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.VisionDrive;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final SendableChooser<Command> autoChooser;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyroButton = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final NetworkTable visionTable;
    private NetworkTableEntry DistanceEntry;
    private double Distance;

    public RobotContainer() {
        JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        DistanceEntry = visionTable.getEntry("Distance");

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
        VisionStop();
    }

    public boolean VisionStop() {
        DistanceEntry = visionTable.getEntry("Distance");
        Distance = DistanceEntry.getDouble(0.0);
        if (Distance == 0.0){
            return false;
        }
        return true;
    }
    private void configureBindings() {
    
        DistanceEntry = visionTable.getEntry("Distance");
        Distance = DistanceEntry.getDouble(0.0);
        
        zeroGyroButton.onTrue(new InstantCommand(swerve::zeroGyro));

        // Run VisionDrive continuously if distance is not zero
        new Thread(() -> {
            while (true) {
                Distance = DistanceEntry.getDouble(0.0);
                if (Distance > 0.01) {
                    new VisionDrive(swerve).schedule();
                    new IntakeCommand().startIntakeMotor(0.8).schedule();
                else{
                    new IntakeCommand().stopIntakeMotor().schedule();
                }
                try {
                    Thread.sleep(100); // Adjust the sleep duration as needed
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();
    }
    

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
