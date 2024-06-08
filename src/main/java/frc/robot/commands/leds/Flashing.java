// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDsConstants;
import frc.robot.subsystems.LEDsSubsystem;

public class Flashing extends Command {
    private final LEDsSubsystem leds;

    private int t = 0;

    public Flashing(LEDsSubsystem leds) {
        this.leds = leds;

        addRequirements(leds);
    }

    @Override
    public void initialize() {
        leds.start();
    }

    @Override
    public void execute() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT);
        Color currentColor = new Color();

        if(t % 5 < 2) {
            currentColor = Color.kRed;
        }

        for (int i = 0; i < LEDsConstants.LEDS_COUNT; i++) {
            buffer.setLED(i, currentColor);
        }

        leds.setBuffer(buffer);
        t++;
    }

    @Override
    public void end(boolean interrupt) {
        leds.stop();
    }
}
