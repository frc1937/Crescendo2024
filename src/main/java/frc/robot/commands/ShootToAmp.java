// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_FORWARD;
import static frc.robot.Constants.Swerve.TRANSLATION_CONTROLLER_CONSTRAINTS;
import static frc.robot.Constants.Swerve.TRANSLATION_CONTROLLER_P;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootToAmp extends SequentialCommandGroup {
  /** Creates a new ShootToAmp. */
  public ShootToAmp(ShooterSubsystem shooter, DrivetrainSubsystem drivetrain) {
    Command prepare = new ParallelCommandGroup(
      new PrepareShooter(shooter, new ShooterSubsystem.Reference(Rotation2d.fromDegrees(100), RPM.of(3000))),
      new DriveForwards(drivetrain)
    ).withTimeout(2);

    Command release = shooter.startEnd(
      () -> shooter.setKickerSpeed(KICKER_SPEED_FORWARD),
      shooter::reset
    ).withTimeout(1);

    addCommands(
      prepare,
      release
    );
  }

  private class DriveForwards extends Command {
    private final DrivetrainSubsystem drivetrain;

    private final ProfiledPIDController controller = new ProfiledPIDController(
      TRANSLATION_CONTROLLER_P, 0, 0, TRANSLATION_CONTROLLER_CONSTRAINTS
    );

    private Translation2d initialPosition;

    public DriveForwards(DrivetrainSubsystem drivetrain) {
      this.drivetrain = drivetrain;

      controller.setGoal(0.1);

      addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
      controller.reset(0);
      initialPosition = drivetrain.getPose().getTranslation();
    }

    @Override
    public void execute() {
      drivetrain.drive(new Translation2d(controller.calculate(getTraveledDistance()), 0), 0, false, true);
    }

    @Override
    public void end(boolean interrupted) {
      drivetrain.stop();
    }

    private double getTraveledDistance() {
      return drivetrain.getPose().getTranslation().getDistance(initialPosition);
    }
  } 
}
