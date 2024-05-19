// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static frc.robot.Constants.INFREQUENT_PERIODIC_HERTZ;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private CommandScheduler commandScheduler;
    private RobotContainer robotContainer;
    Timer gcTimer = new edu.wpi.first.wpilibj.Timer();

    public Robot() {
        addPeriodic(() -> robotContainer.infrequentPeriodic(), 1/ INFREQUENT_PERIODIC_HERTZ);
        addPeriodic(() -> robotContainer.frequentPeriodic(), 1 / Constants.PERIODIC_FREQUENCY_HERTZ, 0.05);
    }

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        commandScheduler = CommandScheduler.getInstance();

        gcTimer.start();
    }

    @Override
    public void robotPeriodic() {
        commandScheduler.run();

        if (gcTimer.advanceIfElapsed(5)) {
            System.gc();
        }
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
