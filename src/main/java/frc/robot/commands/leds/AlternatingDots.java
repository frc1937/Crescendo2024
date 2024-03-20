// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDsConstants;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AlternatingDots extends Command {
    private final LEDsSubsystem leds;
    private final ShooterSubsystem shooter;

    private int t = 0;

    private static final AddressableLEDBuffer emptyState0Buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT),
                                              emptyState1Buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT),
                                              loadedState0Buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT),
                                              loadedState1Buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT);

    static {
        for (int i = 0; i < LEDsConstants.LEDS_COUNT; i++) {
            boolean condition = (i / 3) % 2 == 0;
            emptyState0Buffer.setLED(i, condition ? LEDsConstants.COLOUR_WHEN_EMPTY : new Color8Bit());
            emptyState1Buffer.setLED(i, !condition ? LEDsConstants.COLOUR_WHEN_EMPTY : new Color8Bit());
            loadedState0Buffer.setLED(i, condition ? LEDsConstants.COLOUR_WHEN_LOADED : new Color8Bit());
            loadedState1Buffer.setLED(i, !condition ? LEDsConstants.COLOUR_WHEN_LOADED : new Color8Bit());
        }
    }

    public AlternatingDots(LEDsSubsystem leds, ShooterSubsystem shooter) {
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
        // TODO This might set the same buffer twice but who cares
        if ((t / 5) % 2 == 0) {
            if (shooter.isLoaded()) {
                leds.setBuffer(loadedState0Buffer);
            } else {
                leds.setBuffer(emptyState0Buffer);
            }
        } else {
            if (shooter.isLoaded()) {
                leds.setBuffer(loadedState1Buffer);
            } else {
                leds.setBuffer(emptyState1Buffer);
            }
        }

        t++;
    }

    @Override
    public void end(boolean interrupt) {
        leds.stop();
    }
}
