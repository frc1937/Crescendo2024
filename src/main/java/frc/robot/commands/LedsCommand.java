package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedsSubsystem;


public class LedsCommand extends Command {
    private final LedsSubsystem ledsSubsystem;
    private final String animation;

    public LedsCommand(LedsSubsystem ledsSubsystem, String animation) {
        this.ledsSubsystem = ledsSubsystem;
        this.animation = animation;

        addRequirements(this.ledsSubsystem);
    }

    @Override
    public void initialize() {
        ledsSubsystem.startLEDs(animation);
        System.out.println("----- STARTED LEDS!!!");
    }


    @Override
    public void end(boolean interrupted) {
    }
}
