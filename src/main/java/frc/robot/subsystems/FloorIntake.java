// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_BACKWARDS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FloorIntake extends SequentialCommandGroup {
  /** Creates a new FloorIntake. */
  public FloorIntake(IntakeSubsystem intake, ShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    var lowerShooter = new FunctionalCommand(() -> shooter.setPivotAngle(Rotation2d.fromDegrees(0.5)),
                                             () -> {},
                                             interrupted -> {},
                                             shooter::hasPivotArrived,
                                             shooter);

    addCommands(lowerShooter, new LoweredShooterIntake(intake, shooter));
  }

  private class LoweredShooterIntake extends Command {
    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;
    private int consecutiveNoteInsideSamples;

    public LoweredShooterIntake(IntakeSubsystem intake, ShooterSubsystem shooter) {
      this.intake = intake;
      this.shooter = shooter;

      addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
      consecutiveNoteInsideSamples = 0;
      intake.setSpeedPercentage(0.7);
      shooter.setFlywheelSpeed(-3000, false);
      shooter.setKickerSpeed(KICKER_SPEED_BACKWARDS);
    }

    @Override
    public void execute() {
      if (shooter.doesSeeNote()) {
        ++consecutiveNoteInsideSamples;
      } else {
        consecutiveNoteInsideSamples = 0;
      }
    }

    @Override
    public void end(boolean interrupted) {
      shooter.stopFlywheels();
      intake.stopMotor();
      shooter.stopKicker();
    }

    @Override
    public boolean isFinished() {
      return consecutiveNoteInsideSamples >= 12;  // TODO magic number, move to Constants (it is 250 ms / 2 ms cycles)
    }
  }
}
