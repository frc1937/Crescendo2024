// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDsConstants;
import frc.robot.subsystems.LEDsSubsystem;

@Deprecated
public class OsculatingStrip extends Command {
  private final LEDsSubsystem leds;

  private int t = 0;

  public OsculatingStrip(LEDsSubsystem leds) {
    this.leds = leds;

    addRequirements(leds);
  }

  @Override
  public void execute() {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT);

    int stripMiddle = (int)(30.d * Math.cos(t * 0.05));

    for (int i = 0; i < LEDsConstants.LEDS_COUNT; i++) {
      buffer.setLED(i, Math.abs(stripMiddle - i) < 5 ? LEDsConstants.COLOUR_WHEN_EMPTY : new Color8Bit());
    }

    leds.setBuffer(buffer);

    t++;
  }
}
