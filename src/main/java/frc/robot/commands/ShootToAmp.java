// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve5990;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.constants.Constants.ShootingConstants.AMP_INIT;
import static frc.robot.constants.Constants.ShootingConstants.KICKER_SPEED_FORWARD;

public class ShootToAmp extends SequentialCommandGroup {
    /**
     * Creates a new ShootToAmp.
     */
    public ShootToAmp(ShooterSubsystem shooter, Swerve5990 swerve5990, LEDsSubsystem leds) {
        Command prepare = new ParallelCommandGroup(
                new TeleOpDrive(swerve5990, () -> 0.1, () -> 0, () -> 0, () -> true).withTimeout(0.2),
//                new DriveForward(swerve5990, 0.1).withTimeout(0.2), //0.19
                new PrepareShooter(shooter, AMP_INIT).withTimeout(1.9 + 0.37)
        );

        Command release = new ParallelDeadlineGroup(
            shooter.startEnd(
                    () -> {
                        shooter.setKickerSpeed(KICKER_SPEED_FORWARD);
                        shooter.setFlywheelsSpeed(RPM.of(500));
                    },
                    shooter::reset
            )//,
            //new AlternatingDots(leds, shooter)
        ).withTimeout(1.5);

        addCommands(
                prepare,
                release
        );
    }

    @Deprecated
    private static class DriveForward extends Command {
        private final Swerve5990 swerve5990;
        private final double translationValue;

        public DriveForward(Swerve5990 swerve5990, double translationValue) {
            this.swerve5990 = swerve5990;
            this.translationValue = translationValue;

            addRequirements(swerve5990);
        }

        @Override
        public void execute() {
            swerve5990.drive(
                    -translationValue,
                    0,
                    0, true
            );
        }

        @Override
        public void end(boolean interrupt) {
            swerve5990.stop();
        }
    }
}
