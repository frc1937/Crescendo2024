// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AdjustShooter;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Navigate;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.ShooterKick;
import frc.robot.commands.TeleopShooting;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ShootingStates;
import frc.robot.util.TriggerButton;

import static frc.robot.Constants.ShootingConstants.SHOOTING_DELAY;

public class RobotContainer {
    private Thread visionThread;
    private final XboxController driveController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    private final SendableChooser<Command> autoChooser;

    /* MAIN-DRIVER */
    private final JoystickButton startButton = new JoystickButton(driveController, XboxController.Button.kStart.value);
    private final TriggerButton leftTrigger = new TriggerButton(driveController, XboxController.Axis.kLeftTrigger);
    private final TriggerButton rightTrigger = new TriggerButton(driveController, XboxController.Axis.kRightTrigger);
    private final JoystickButton aButton = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton leftBumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton backButton = new JoystickButton(driveController, XboxController.Button.kBack.value);
    // private final JoystickButton povUp = new JoystickButton(driveController, XboxController.Button.);
    /* OPERATOR */
    private final TriggerButton accelerateFlywheelButton = new TriggerButton(operatorController, XboxController.Axis.kRightTrigger);
    private final JoystickButton randomPitchYButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
    private final JoystickButton opAButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
    private final JoystickButton opBButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
    private final JoystickButton opYButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
    private final JoystickButton opXButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
    private final JoystickButton opAccelerateButton = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton opStartButton = new JoystickButton(operatorController, XboxController.Button.kStart.value);
    /* Subsystems */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    /* Commands */
    private final ShooterCommands shooterCommands = new ShooterCommands(shooterSubsystem, intakeSubsystem);
    private final NetworkTable visionTable;
    private NetworkTableEntry DistanceEntry;
    private boolean operatorShot = false;

    public RobotContainer() {
        visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        DistanceEntry = visionTable.getEntry("Distance");

        JoystickButton robotCentric = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);

        swerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        swerveSubsystem,
                        () -> -driveController.getRawAxis(XboxController.Axis.kLeftY.value),
                        () -> -driveController.getRawAxis(XboxController.Axis.kLeftX.value),
                        () -> -driveController.getRawAxis(XboxController.Axis.kRightX.value),
                        robotCentric
                )
        );

        NamedCommands.registerCommand("Shifra2", shooterCommands.intakeGet().withTimeout(5));
        NamedCommands.registerCommand("AdjustShooter22", new AdjustShooter(shooterSubsystem, 0.8));
        NamedCommands.registerCommand("AdjustShooter23", new AdjustShooter(shooterSubsystem, 0.8));
        // NamedCommands.registerCommand("TeleopShooting", new TeleopShooting(swerveSubsystem, shooterSubsystem, () -> 0, () -> 0).withTimeout(4));
        NamedCommands.registerCommand("AdjustShooter1", new AdjustShooter(shooterSubsystem, 1.15));
        NamedCommands.registerCommand("AdjustShooter2", new AdjustShooter(shooterSubsystem, 0.9));
        NamedCommands.registerCommand("AdjustShooter3", new AdjustShooter(shooterSubsystem, 0.8));
        NamedCommands.registerCommand("AdjustShooter4", new AdjustShooter(shooterSubsystem, 0.9));
        NamedCommands.registerCommand("ShooterKick", new ShooterKick(shooterSubsystem).withTimeout(SHOOTING_DELAY));

        autoChooser = AutoBuilder.buildAutoChooser("Eyal");
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }


    private void configureBindings() {
        //TODO: Uriel define this commands to whichever buttons you want
        // Teleop shooting automaticy audujsts itself based on april tag for shooting
        // accelerate flywheel is the command that was asked by Ofir
        backButton.whileTrue(Navigate.navigateToAmplifier());

        startButton.onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        leftTrigger.whileTrue(shooterCommands.intakeGet()
                .andThen(shooterCommands.setKickerSpeed(-0.8)
                .withTimeout(0.7)));

        aButton.whileTrue(
                new TeleopShooting(swerveSubsystem, shooterSubsystem,
                () -> -driveController.getRawAxis(XboxController.Axis.kLeftY.value),
                        () -> -driveController.getRawAxis(XboxController.Axis.kLeftX.value))

        );

        leftBumper.whileTrue(shooterCommands.receiveFromFeeder().andThen(shooterCommands.setKickerSpeed(-0.8)
                .withTimeout(0.7)));

       // accelerateFlywheelButton.whileTrue(shooterCommands.accelerateFlywheel());

        rightTrigger.whileTrue(new IntakeCommand(intakeSubsystem, -0.9));
        opAccelerateButton.onTrue(!operatorShot ? new ShooterKick(shooterSubsystem) : new AdjustShooter(shooterSubsystem, 1.15));
        

        //sagi:
        opAButton.whileTrue(shooterCommands.shootNote(ShootingStates.SPEAKER_FRONT));
        opBButton.whileTrue(shooterCommands.shootNote(ShootingStates.SPEAKER_BACK));
        opYButton.whileTrue(shooterCommands.shootNote(ShootingStates.AMP));
        opXButton.whileTrue(shooterCommands.shootNote(ShootingStates.STAGE_FRONT));
        

//        aButton.whileTrue(shooterCommands.setAngle(60));
//
//        aButton.whileTrue(new TeleopShooting(swerveSubsystem, shooterSubsystem,
//                () -> -driver.getRawAxis(XboxController.Axis.kLeftY.value),
//                () -> -driver.getRawAxis(XboxController.Axis.kLeftX.value)));

//
//        DistanceEntry = visionTable.getEntry("Distance");
//        DistanceEntry.getDouble(0.0);
//
////        accelerateFlywheelButton.whileTrue(
////                new ParallelCommandGroup(
////                        new VisionDrive(swerveSubsystem),
////                        shooterCommands.intakeGet()
////                )
////        );
//
//      //  accelerateFlywheelButton.onFalse(new InstantCommand(intakeSubsystem::stopMotor));
    }

    //todo:
    // Faster pitch
    // quasistatic table of values
    // max acc, max speed, (Ask CAD people), all these goofy constants
    //

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void infrequentPeriodic() {
        swerveSubsystem.infrequentPeriodic();
    }

    public void robotInit() {
        visionThread = new Thread(() -> {
            UsbCamera camera = CameraServer.startAutomaticCapture(0);

            camera.setResolution(480, 270);
            camera.setFPS(30);
        });

        visionThread.setDaemon(true);
        visionThread.start();
    }
}
