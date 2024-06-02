package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.AlliancePose2d;
import frc.robot.Constants;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterPhysicsCalculations;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve5990;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.VisionConstants.BLUE_SPEAKER;
import static frc.robot.Constants.VisionConstants.RED_SPEAKER;
import static frc.robot.subsystems.shooter.ShooterConstants.INTAKE;
import static frc.robot.subsystems.shooter.ShooterConstants.KICKER_SPEED_BACKWARDS;
import static frc.robot.subsystems.shooter.ShooterConstants.KICKER_SPEED_FORWARD;

public class ShooterCommands {
    private final ShooterSubsystem shooterSubsystem;
    private final LEDsSubsystem leds;
    private final Swerve5990 swerve5990;

    private final IntakeCommands intakeCommands;
    private final ShooterPhysicsCalculations shooterPhysicsCalculations;

    private final StructArrayPublisher<Pose3d> targetPoses = NetworkTableInstance.getDefault()
            .getStructArrayTopic("TargetPoses", Pose3d.struct).publish();

    public ShooterCommands(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, LEDsSubsystem leds, Swerve5990 swerve5990, ShooterPhysicsCalculations shooterPhysicsCalculations) {
        this.shooterSubsystem = shooterSubsystem;
        this.leds = leds;
        this.swerve5990 = swerve5990;
        this.shooterPhysicsCalculations = shooterPhysicsCalculations;

        intakeCommands = new IntakeCommands(intakeSubsystem);
    }

    public Command shootPhysics(double tangentialVelocity) {
        Pose3d targetPose = AlliancePose2d.AllianceUtils.isBlueAlliance() ? BLUE_SPEAKER : RED_SPEAKER;

        targetPoses.set(
                new Pose3d[]{
                        targetPose,
                        Constants.VisionConstants.TAG_ID_TO_POSE.get(7)
                }
        );

        return new FunctionalCommand(
                () -> {
                    Rotation2d theta = shooterPhysicsCalculations.getPitchAnglePhysics(targetPose, tangentialVelocity);

                    ShooterSubsystem.Reference reference = new ShooterSubsystem.Reference(
                            theta, MetersPerSecond.of(tangentialVelocity)
                    );

                    SmartDashboard.putString("physics/targetPose", targetPose.toString());
                    SmartDashboard.putNumber("physics/theta", theta.getDegrees());

                    Rotation2d targetAngle = shooterPhysicsCalculations.getAzimuthAngleToTarget(targetPose);

                    swerve5990.driveWithTargetAzimuth(0, 0, targetAngle);
                    initializeShooter(false, reference);
                },
                () -> {
                    SmartDashboard.putBoolean("shooter/isLoaded", shooterSubsystem.isLoaded());
                    SmartDashboard.putBoolean("shooter/flywheelsAtReference", shooterSubsystem.flywheelsAtReference());
                    SmartDashboard.putBoolean("shooter/pitchAtReference", shooterSubsystem.pitchAtReference());

                    if (shooterSubsystem.isLoaded() && shooterSubsystem.flywheelsAtReference() && shooterSubsystem.pitchAtReference()) {
                        shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD);
                    }
                },
                interrupted -> shooterSubsystem.reset(),
                () -> false,

                shooterSubsystem
        );
    }

    public Command shootNote(ShooterSubsystem.Reference reference) {
        return new FunctionalCommand(
                () -> initializeShooter(false, reference),
                () -> {
                    SmartDashboard.putBoolean("shooter/isLoaded", shooterSubsystem.isLoaded());
                    SmartDashboard.putBoolean("shooter/flywheelsAtReference", shooterSubsystem.flywheelsAtReference());
                    SmartDashboard.putBoolean("shooter/pitchAtReference", shooterSubsystem.pitchAtReference());

                    if (shooterSubsystem.isLoaded() && shooterSubsystem.flywheelsAtReference() && shooterSubsystem.pitchAtReference()) {
                        shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD);
                    }
                },
                interrupted -> shooterSubsystem.reset(),
                () -> false,

                shooterSubsystem
        );
    }

    public Command setPitchPosition(double degrees) {
        return new FunctionalCommand(
                () -> initializeShooter(false, new ShooterSubsystem.Reference(Rotation2d.fromDegrees(degrees))),
                () -> {},
                interrupted -> {},
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
