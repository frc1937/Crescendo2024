// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs;
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        ctreConfigs = new CTREConfigs();
        robotContainer = new RobotContainer();
    
        try {
            System.out.println("Starting VisionDataReceiver...");
            VisionDataReceiver.main(new String[]{});
            System.out.println("VisionDataReceiver started successfully.");
        } catch (Exception e) {
            System.err.println("Error starting VisionDataReceiver:");
            e.printStackTrace();
        }
    }
    

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
/*
        System.out.println("-----------------------------------------");
        System.out.println("Has Target: " + poseEstimator.hasTarget());

        if (poseEstimator.getBestTargetID() != null)
            System.out.println("Target ID: " + poseEstimator.getBestTargetID());
   */
    }

    @Override
    public void disabledInit() {
    }


    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }


    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
