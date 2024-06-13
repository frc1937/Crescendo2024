package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterPhysicsCalculations;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve5990;

import static frc.robot.subsystems.shooter.ShooterConstants.INTAKE;
import static frc.robot.subsystems.shooter.ShooterConstants.KICKER_SPEED_BACKWARDS;
import static frc.robot.subsystems.shooter.ShooterConstants.KICKER_SPEED_FORWARD;

public class ShooterCommands {
    private final ShooterSubsystem shooterSubsystem;
    private final LEDsSubsystem leds;
    private final Swerve5990 swerve5990;

    private final IntakeCommands intakeCommands;
    private final ShooterPhysicsCalculations shooterPhysicsCalculations;

    private final StructArrayPublisher<Pose3d> targetPoses = NetworkTableInstance.getDefault().getStructArrayTopic("TargetPoses", Pose3d.struct).publish();

    public ShooterCommands(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, LEDsSubsystem leds, Swerve5990 swerve5990, ShooterPhysicsCalculations shooterPhysicsCalculations) {
        this.shooterSubsystem = shooterSubsystem;
        this.leds = leds;
        this.swerve5990 = swerve5990;
        this.shooterPhysicsCalculations = shooterPhysicsCalculations;

        intakeCommands = new IntakeCommands(intakeSubsystem);
    }

    public Command shootPhysics(double tangentialVelocity) {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.resetController();
                },
                () -> {
                    shooterPhysicsCalculations.updateValuesForSpeakerAlignment(swerve5990.getSelfRelativeVelocity());

                    targetPoses.set(new Pose3d[]{
                                    shooterPhysicsCalculations.getTargetPose(),
                                    Constants.VisionConstants.TAG_ID_TO_POSE.get(7)
                            }
                    );

                    Rotation2d targetPitchAngle = shooterPhysicsCalculations.getPitchAnglePhysics();
                    Rotation2d targetAzimuthAngle = shooterPhysicsCalculations.getAzimuthAngleToTarget();

                    shooterSubsystem.setReference(new ShooterSubsystem.Reference(targetPitchAngle,
                    tangentialVelocity
                    ));
                    swerve5990.driveFieldRelative(0, 0, targetAzimuthAngle);

                    SmartDashboard.putNumber("shooter/physicsAngle [DEG]", targetPitchAngle.getDegrees());

                    logShooterConditions();

                    if (hasMetShootingConditions())
                        shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD);
                },
                interrupted -> shooterSubsystem.reset(),
                () -> false,

                shooterSubsystem
        );
    }

    public Command shootNote(ShooterSubsystem.Reference reference) {
        return new FunctionalCommand(
                () -> shooterSubsystem.setReference(reference),
                () -> {
                    logShooterConditions();

                    if (hasMetShootingConditions())
                        shooterSubsystem.setKickerSpeed(KICKER_SPEED_FORWARD);
                },
                interrupted -> {
                    shooterSubsystem.reset();
                },
                () -> false,

                shooterSubsystem
        );
    }

    public Command setPitchPosition(double degrees) {
        return new FunctionalCommand(
                () -> shooterSubsystem.setReference(new ShooterSubsystem.Reference(Rotation2d.fromDegrees(degrees))),
                () -> {
                },
                interrupted -> {
                },
                () -> false,

                shooterSubsystem
        );
    }

    public Command postIntake() {
        return new SequentialCommandGroup(new FunctionalCommand(
                () -> {
                    shooterSubsystem.setTangentialFlywheelsVelocity(-6);
                    shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
                },
                () -> {
                },
                interrupted -> shooterSubsystem.reset(),
                () -> false,

                shooterSubsystem
        ).withTimeout(0.4)).alongWith(intakeCommands.stopIntake(0.4));
    }

    public Command floorIntake() {
        return new FunctionalCommand(
                () -> {
                    shooterSubsystem.setReference(INTAKE);
                    shooterSubsystem.setKickerSpeed(KICKER_SPEED_BACKWARDS);
                },
                () -> {
                },
                interrupted -> {
                },
                shooterSubsystem::isLoaded,

                shooterSubsystem
        ).alongWith(intakeCommands.enableIntake(0.8, false));
    }

    private boolean hasMetShootingConditions() {
        return shooterSubsystem.isLoaded() && shooterSubsystem.flywheelsAtReference() && shooterSubsystem.pitchAtReference();
    }

    private void logShooterConditions() {
        SmartDashboard.putBoolean("shooter/isLoaded", shooterSubsystem.isLoaded());
        SmartDashboard.putBoolean("shooter/flywheelsAtReference", shooterSubsystem.flywheelsAtReference());
        SmartDashboard.putBoolean("shooter/pitchAtReference", shooterSubsystem.pitchAtReference());
    }
}
