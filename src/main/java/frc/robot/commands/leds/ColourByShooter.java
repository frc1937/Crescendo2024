// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDsConstants;
import frc.robot.subsystems.LEDsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ColourByShooter extends Command {
    private final LEDsSubsystem leds;
    private final ShooterSubsystem shooter;

    private boolean wasLoaded = false;

    private static final AddressableLEDBuffer bufferWhenEmpty = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT);
    private static final AddressableLEDBuffer bufferWhenLoaded = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT);

    static {
        for (int i = 0; i < LEDsConstants.LEDS_COUNT; i++) {
            bufferWhenEmpty.setLED(i, LEDsConstants.COLOUR_WHEN_EMPTY);
        }

        for (int i = 0; i < LEDsConstants.LEDS_COUNT; i++) {
            bufferWhenLoaded.setLED(i, LEDsConstants.COLOUR_WHEN_LOADED);
        }
    }

    public ColourByShooter(LEDsSubsystem leds, ShooterSubsystem shooter) {
        this.leds = leds;
        this.shooter = shooter;

        addRequirements(leds);
    }

    @Override
    public void initialize() {
        leds.setBuffer(bufferWhenEmpty);
    }

    @Override
    public void execute() {
        if (wasLoaded != shooter.isLoaded()) {
            wasLoaded = shooter.isLoaded();

            if (wasLoaded) {
                leds.setBuffer(bufferWhenLoaded);
            } else {
                leds.setBuffer(bufferWhenEmpty);
            }
        }
    }

    @Override
    public void end(boolean interrupt) {
        leds.clear();
    }
}
