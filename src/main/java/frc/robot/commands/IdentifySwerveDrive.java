// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.SwerveSubsystem;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class IdentifySwerveDrive extends SequentialCommandGroup {
  public IdentifySwerveDrive(SwerveSubsystem swerve) {
    SysIdRoutine routine = new SysIdRoutine(
      new Config(Volts.per(Second).of(0.75), null, Seconds.of(50)),
      new Mechanism(swerve::rotateByVoltage, swerve::logAllModulesDrive, swerve));
    
      // Rotate the swerve s.t. the rest of the tests will be performed by rotation
      Command prepareSwerveModuleAngle = new TeleopSwerve(swerve, () -> 0, () -> 0, () -> 1, () -> false);

        addCommands(prepareSwerveModuleAngle.withTimeout(0.5),
                routine.quasistatic(Direction.kForward), routine.quasistatic(Direction.kReverse),
                routine.dynamic(Direction.kForward), routine.dynamic(Direction.kReverse));
    }
}
