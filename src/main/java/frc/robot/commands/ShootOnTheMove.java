package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.poseestimation.PoseEstimator5990;
import frc.robot.subsystems.shooter.ShooterPhysicsCalculations;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve5990;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.swerve.SwerveConstants.SWERVE_AZIMUTH_TOLERANCE;

public class ShootOnTheMove extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final ShooterCommands shooterCommands;
    private final Swerve5990 swerve5990;
    private final PoseEstimator5990 poseEstimator5990;
    private final ShooterPhysicsCalculations shooterPhysicsCalculations;

    private final DoubleSupplier translationSupplier, strafeSupplier;
    private final double tangentialVelocity;

    public ShootOnTheMove(ShooterSubsystem shooterSubsystem, ShooterCommands shooterCommands, Swerve5990 swerve5990, PoseEstimator5990 poseEstimator5990, DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier, double tangentialVelocity, ShooterPhysicsCalculations shooterPhysicsCalculations) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterCommands = shooterCommands;
        this.swerve5990 = swerve5990;
        this.poseEstimator5990 = poseEstimator5990;

        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.tangentialVelocity = tangentialVelocity;

        this.shooterPhysicsCalculations = shooterPhysicsCalculations;
    }

    @Override
    public void execute() {
        shooterPhysicsCalculations.updateValuesForSpeakerAlignment(swerve5990.getSelfRelativeVelocity());

        double targetTranslation = MathUtil.applyDeadband(translationSupplier.getAsDouble(), Constants.STICK_DEADBAND);
        double targetStrafe = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), Constants.STICK_DEADBAND);

        Rotation2d targetAzimuthAngle = shooterPhysicsCalculations.getAzimuthAngleToTarget();

        swerve5990.driveFieldRelative(targetTranslation, targetStrafe, targetAzimuthAngle);
        shootNote(tangentialVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooterSubsystem.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return swerve5990.azimuthAtGoal(Radians.of(SWERVE_AZIMUTH_TOLERANCE)) && shooterSubsystem.atReference();
    }

    private void shootNote(double tangentialVelocity) {
        Rotation2d theta = shooterPhysicsCalculations.getTargetAnglePhysics();
        ShooterSubsystem.Reference reference = new ShooterSubsystem.Reference(theta, tangentialVelocity);

        shooterCommands.shootNote(reference);
    }
}
