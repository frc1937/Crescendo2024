package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterPhysicsCalculations;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.AlliancePose2d;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.BLUE_SPEAKER;
import static frc.robot.Constants.RED_SPEAKER;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterCommands {
    private final ShooterSubsystem shooterSubsystem;
    private final LEDsSubsystem leds;
    private final IntakeSubsystem intakeSubsystem;
    private final PoseEstimator5990 poseEstimator5990;

    private final ShooterPhysicsCalculations shooterPhysicsCalculations;

    public ShooterCommands(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, LEDsSubsystem leds, PoseEstimator5990 poseEstimator5990) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.leds = leds;
        this.poseEstimator5990 = poseEstimator5990;

        shooterPhysicsCalculations = new ShooterPhysicsCalculations(shooterSubsystem);
    }

    public Command shootPhysics(double tangentialVelocity) {
        Pose3d targetPose = AlliancePose2d.AllianceUtils.isBlueAlliance() ? BLUE_SPEAKER : RED_SPEAKER;
        Pose2d robotPose = poseEstimator5990.getCurrentPose().getCorrectPose();

        Rotation2d theta = shooterPhysicsCalculations.getPitchAnglePhysics(robotPose, targetPose, tangentialVelocity);
        ShooterSubsystem.Reference reference = new ShooterSubsystem.Reference(theta, MetersPerSecond.of(tangentialVelocity));

        return shootNote(reference);
    }

    public Command shootNote(ShooterSubsystem.Reference reference) {
        return new FunctionalCommand(
                () -> initializeShooter(false, reference),
                () -> {
                    if (shooterSubsystem.isLoaded() && shooterSubsystem.flywheelsAtReference() && shooterSubsystem.pitchAtReference()) {
                        shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD);
                    }
                },
                interrupted -> shooterSubsystem.reset(),
                () -> false,

                shooterSubsystem
        );
    }

    public Command postIntake() {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.setFlywheelsSpeed(RPM.of(-2000));
                    shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
                    intakeSubsystem.setSpeedPercentage(0.4);
                },
                () -> {},
                interrupted -> {
                    shooterSubsystem.reset();
                    intakeSubsystem.stop();
                },
                () -> false,
                shooterSubsystem
        );
    }

    public Command floorIntake() {
        return new ParallelDeadlineGroup(
            new FunctionalCommand(
                () -> {
                    initializeShooter(true, INTAKE);
                    intakeSubsystem.setSpeedPercentage(0.8);
                },
                () -> {},
                interrupted -> {},
                shooterSubsystem::isLoaded,
                shooterSubsystem
            )//,
            //new OsculatingStrip(leds, shooterSubsystem)
        );
    }

    private void initializeShooter(boolean shouldUseKicker, ShooterSubsystem.Reference reference) {
        if(shouldUseKicker)
            shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);

        shooterSubsystem.setReference(reference);
    }
}
