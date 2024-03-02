// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.ShooterSubsystem;

/** Add your docs here. */
public class PivotSysId {
    private final SysIdRoutine routine;

    public PivotSysId(ShooterSubsystem shooter) {
        routine = new SysIdRoutine(
            new Config(Volts.per(Seconds).of(0.3), Volts.of(3), Seconds.of(10)),
            new Mechanism(shooter::setPitchVoltage, shooter::logPitch, shooter));
    }

    public Command getQuasistaticTest(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command getDynamicTest(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
