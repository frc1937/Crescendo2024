// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDsConstants;
import frc.robot.subsystems.LEDsSubsystem;

public class AlternatingDots extends Command {
    private final LEDsSubsystem leds;

    private boolean state = false;

    private static final AddressableLEDBuffer state0Buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT),
            state1Buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT);

    static {
        for (int i = 0; i < LEDsConstants.LEDS_COUNT; i++) {
            state0Buffer.setLED(i, i % 2 == 0 ? LEDsConstants.COLOUR_WHEN_EMPTY : new Color8Bit());
            state1Buffer.setLED(i, i % 2 == 1 ? LEDsConstants.COLOUR_WHEN_EMPTY : new Color8Bit());
        }
    }

    public AlternatingDots(LEDsSubsystem leds) {
        this.leds = leds;

        addRequirements(leds);
    }

    @Override
    public void initialize() {
        leds.start();
    }

    @Override
    public void execute() {
        if (state) {
            leds.setBuffer(state0Buffer);
        } else {
            leds.setBuffer(state1Buffer);
        }

        state = !state;
    }

    @Override
    public void end(boolean interrupt) {
        leds.stop();
    }
}
