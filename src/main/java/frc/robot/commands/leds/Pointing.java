// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDsConstants;
import frc.robot.subsystems.LEDsSubsystem;

public class Pointing extends Command {
  private final LEDsSubsystem leds;

  private int t = 0;

  public Pointing(LEDsSubsystem leds) {
    this.leds = leds;

    addRequirements(leds);
  }

  @Override
  public void execute() {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT);

    int slowT = (int)((double)t / 1.5d);
    int stripMiddle = slowT % (LEDsConstants.LEDS_COUNT / 4);

    for (int i = 0; i < LEDsConstants.LEDS_COUNT / 4; i++) {
      Color8Bit colour = Math.abs(stripMiddle - i) < 3 ? LEDsConstants.COLOUR_WHEN_LOADED : new Color8Bit();
      buffer.setLED(i, colour);
      buffer.setLED(LEDsConstants.LEDS_COUNT / 2 - i - 1, colour);
      buffer.setLED(LEDsConstants.LEDS_COUNT / 2 + i, colour);
      buffer.setLED(LEDsConstants.LEDS_COUNT - i - 1, colour);
    }

    leds.setBuffer(buffer);

    t++;
  }
}
