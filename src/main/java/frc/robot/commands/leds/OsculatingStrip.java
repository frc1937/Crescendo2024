// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDsConstants;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class OsculatingStrip extends Command {
    private final LEDsSubsystem leds;
    private final ShooterSubsystem shooter;
    private int t = 0;
    public OsculatingStrip(LEDsSubsystem leds, ShooterSubsystem shooter) {
        this.leds = leds;
        this.shooter = shooter;

        addRequirements(leds);
    }

    @Override
    public void initialize() {
        leds.start();
    }

    @Override
    public void execute() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT);

        int stripMiddle = (int) (30.d * Math.cos(t * 0.05));

        Color8Bit globalColour = shooter.isLoaded() ? LEDsConstants.COLOUR_WHEN_LOADED : LEDsConstants.COLOUR_WHEN_EMPTY;

        for (int i = 0; i < LEDsConstants.LEDS_COUNT / 2; i++) {
            Color8Bit localColour = Math.abs(stripMiddle - i) < 5 ? globalColour : new Color8Bit();
            buffer.setLED(i, localColour);
            buffer.setLED(LEDsConstants.LEDS_COUNT - i - 1, localColour);
        }

        leds.setBuffer(buffer);

        t++;
    }

    @Override
    public void end(boolean interrupt) {
        leds.stop();
    }
}
