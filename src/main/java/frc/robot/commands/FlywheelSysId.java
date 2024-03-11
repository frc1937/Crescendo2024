// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Seconds;

/** Add your docs here. */
public class FlywheelSysId {
    private final SysIdRoutine routine;

    public FlywheelSysId(ShooterSubsystem shooter) {
        this.routine = new SysIdRoutine(
            new Config(null, null, Seconds.of(15)),
            new Mechanism(shooter::setFlywheelsVoltage, shooter::logFlywheels, shooter)
        );
    }

    public Command getQuasistaticTest() {
        return routine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command getQuasistaticTestBackward() {
        return routine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command getDynamicTest() {
        return routine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command getDynamicTestBackward() {
        return routine.dynamic(SysIdRoutine.Direction.kReverse);
    }
}
