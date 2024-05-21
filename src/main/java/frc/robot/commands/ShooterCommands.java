package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.AlliancePose2d;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterPhysicsCalculations;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.VisionConstants.BLUE_SPEAKER;
import static frc.robot.Constants.VisionConstants.RED_SPEAKER;
import static frc.robot.subsystems.shooter.ShooterConstants.INTAKE;
import static frc.robot.subsystems.shooter.ShooterConstants.KICKER_SPEED_BACKWARDS;
import static frc.robot.subsystems.shooter.ShooterConstants.KICKER_SPEED_FORWARD;

public class ShooterCommands {
    private final ShooterSubsystem shooterSubsystem;
    private final LEDsSubsystem leds;
    private final PoseEstimator5990 poseEstimator5990;

    private final IntakeCommands intakeCommands;
    private final ShooterPhysicsCalculations shooterPhysicsCalculations;

    public ShooterCommands(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, LEDsSubsystem leds, PoseEstimator5990 poseEstimator5990) {
        this.shooterSubsystem = shooterSubsystem;
        this.leds = leds;
        this.poseEstimator5990 = poseEstimator5990;

        shooterPhysicsCalculations = new ShooterPhysicsCalculations(shooterSubsystem);
        intakeCommands = new IntakeCommands(intakeSubsystem);
    }

    public Command shootPhysics(double tangentialVelocity) {
        Pose3d targetPose = AlliancePose2d.AllianceUtils.isBlueAlliance() ? BLUE_SPEAKER : RED_SPEAKER;
        Pose2d robotPose = poseEstimator5990.getCurrentPose().getBluePose();

        Rotation2d theta = shooterPhysicsCalculations.getPitchAnglePhysics(robotPose, targetPose, tangentialVelocity);
        ShooterSubsystem.Reference reference = new ShooterSubsystem.Reference(theta, MetersPerSecond.of(0)
                //MetersPerSecond.of(tangentialVelocity)
        );

        System.out.println("Button is pressed");

        SmartDashboard.putString("physics/targetPose", targetPose.toString());
        SmartDashboard.putString("physics/robotPose", robotPose.toString());
        SmartDashboard.putNumber("physics/theta", theta.getDegrees());
        SmartDashboard.putNumber("physics/tangentialVelocity", tangentialVelocity);

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
        return new SequentialCommandGroup(new FunctionalCommand(
                () -> {
                    shooterSubsystem.setTangentialFlywheelsVelocity(MetersPerSecond.of(-2));
                    shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
                },
                () -> {},
                interrupted -> shooterSubsystem.reset(),
                () -> false,
                shooterSubsystem
        )).alongWith(intakeCommands.stopIntake(0.4));
    }

    public Command floorIntake() {
        return new FunctionalCommand(
                () -> {
                    initializeShooter(true, INTAKE);
                    intakeCommands.enableIntake(0.8, false).schedule();
                },
                () -> {},
                interrupted -> {},
                shooterSubsystem::isLoaded,
                shooterSubsystem
            );
    }

    private void initializeShooter(boolean shouldUseKicker, ShooterSubsystem.Reference reference) {
        if(shouldUseKicker)
            shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);

        shooterSubsystem.setReference(reference);
    }
}
