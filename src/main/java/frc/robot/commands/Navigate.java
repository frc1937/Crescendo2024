// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.NavigationConstants;

public class Navigate {
    public static Command navigateToAmplifier() {
        return AutoBuilder.pathfindToPoseFlipped(NavigationConstants.AMPLIFIER_POSE, NavigationConstants.DEFAULT_PATH_CONSTRAINTS);
    }
}
