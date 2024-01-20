package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SpinIntake extends Command {

    private final Intake intakeSubsystem;
    private final double speed;

    public SpinIntake(Intake intakeSubsystem, double speed, double duration) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;

        // Use addRequirements() here to declare subsystem dependencies
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        // Called when the command is initially scheduled.
        // You can perform any setup here.
    }

    @Override
    public void execute() {
        // Called repeatedly when this Command is scheduled to run.
        intakeSubsystem.spin(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Called once the command ends or is interrupted.
        intakeSubsystem.stop();
    }
}