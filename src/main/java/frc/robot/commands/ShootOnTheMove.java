package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve5990;
import frc.robot.util.AlliancePose2d;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.BLUE_SPEAKER;
import static frc.robot.Constants.RED_SPEAKER;
import static frc.robot.subsystems.shooter.ShooterConstants.KICKER_SPEED_FORWARD;

public class ShootOnTheMove {
    private final ShooterSubsystem shooterSubsystem;
    private final PoseEstimator5990 poseEstimator5990;
    private final ShooterCommands shooterCommands;
    private final Swerve5990 swerve5990;

    public ShootOnTheMove(ShooterSubsystem shooterSubsystem, PoseEstimator5990 poseEstimator5990, ShooterCommands shooterCommands, Swerve5990 swerve5990) {
        this.shooterSubsystem = shooterSubsystem;
        this.poseEstimator5990 = poseEstimator5990;
        this.shooterCommands = shooterCommands;
        this.swerve5990 = swerve5990;
    }

    public void shootOnTheMove(double tangentialVelocity) {
        //TODO: In no place here do you rotate towards the target

        Pose3d targetPose = AlliancePose2d.AllianceUtils.isBlueAlliance() ? BLUE_SPEAKER : RED_SPEAKER;
        Pose2d robotPose = poseEstimator5990.getCurrentPose().getCorrectPose();

        ChassisSpeeds robotVelocity = swerve5990.getSelfRelativeVelocity();
        double timeOfFlight = shooterSubsystem.getTimeOfFlight(robotPose, targetPose, tangentialVelocity);

        Transform3d targetOffset = new Transform3d(
                robotVelocity.vxMetersPerSecond * timeOfFlight,
                robotVelocity.vyMetersPerSecond * timeOfFlight,
                0,
                new Rotation3d()
        );

        Pose3d newTarget = targetPose.transformBy(targetOffset.inverse());

        shootNote(robotPose, newTarget, tangentialVelocity);
    }

    public Command shootNote(ShooterSubsystem.Reference reference) {
        return new FunctionalCommand(
                () -> shooterSubsystem.setReference(reference),
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

    private void shootNote(Pose2d robotPose, Pose3d targetPose, double tangentialVelocity) {
        Rotation2d theta = shooterSubsystem.getPitchAnglePhysics(robotPose, targetPose, tangentialVelocity);
        ShooterSubsystem.Reference reference = new ShooterSubsystem.Reference(theta, MetersPerSecond.of(tangentialVelocity));

        shooterCommands.shootNote(reference);
    }
}
