// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static frc.robot.Constants.INFREQUENT_PERIODIC_PERIOD;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private CommandScheduler commandScheduler;
    private RobotContainer robotContainer;

    public Robot() {
        addPeriodic(() -> robotContainer.infrequentPeriodic(), INFREQUENT_PERIODIC_PERIOD);
        addPeriodic(() -> robotContainer.frequentPeriodic(), Constants.FREQUENT_PERIODIC_PERIOD, 0.004);
    }

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        commandScheduler = CommandScheduler.getInstance();
    }

    @Override
    public void robotPeriodic() {
        commandScheduler.run();
    }


    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        commandScheduler.cancelAll();
    }
}
