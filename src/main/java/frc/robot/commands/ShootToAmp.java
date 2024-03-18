// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.ShootingConstants.AMP_INIT;
import static frc.robot.Constants.ShootingConstants.KICKER_SPEED_FORWARD;

public class ShootToAmp extends SequentialCommandGroup {
    /**
     * Creates a new ShootToAmp.
     */
    public ShootToAmp(ShooterSubsystem shooter, DrivetrainSubsystem drivetrainSubsystem) {
        Command prepare = new ParallelCommandGroup(
                new DriveForward(drivetrainSubsystem, 0.1).withTimeout(0.17), //0.19
                new PrepareShooter(shooter, AMP_INIT).withTimeout(1.9 + 0.37)
        );

        Command release = shooter.startEnd(
                () -> {
                    shooter.setKickerSpeed(KICKER_SPEED_FORWARD);
                    shooter.setFlywheelsSpeed(RPM.of(600));
                },
                shooter::reset
        ).withTimeout(1.5);

        addCommands(
                prepare,
                release
        );
    }

    @Deprecated
    private static class DriveForward extends Command {
        private final DrivetrainSubsystem drivetrain;
        private final double translationValue;

        public DriveForward(DrivetrainSubsystem drivetrain, double translationValue) {
            this.drivetrain = drivetrain;
            this.translationValue = translationValue;

            addRequirements(drivetrain);
        }

        @Override
        public void execute() {
            drivetrain.drive(
                    new Translation2d(-translationValue, 0).times(Constants.Swerve.MAX_SPEED),
                    0,
                    false,
                    true);
        }

        @Override
        public void end(boolean interrupt) {
            drivetrain.stop();
        }
    }
}
