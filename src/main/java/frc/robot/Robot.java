// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.photonvision.PoseEstimator;
import org.photonvision.PhotonCamera;

import java.util.logging.Logger;

public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs;
    private Command autonomousCommand;
    private PoseEstimator poseEstimator;
    private final PhotonCamera photonCamera = new PhotonCamera("Photon1937");
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        ctreConfigs = new CTREConfigs();
        robotContainer = new RobotContainer();
        poseEstimator = new PoseEstimator(photonCamera);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Logger.getGlobal().info("--------------------------------");
        Logger.getGlobal().info("Has Target: " + poseEstimator.hasTarget());

        if(poseEstimator.getBestTargetID() != null) {
            Logger.getGlobal().info("Target ID: " + poseEstimator.getBestTargetID());
        }
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
