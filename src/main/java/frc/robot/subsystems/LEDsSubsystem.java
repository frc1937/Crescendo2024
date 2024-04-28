package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDsConstants;

public class LEDsSubsystem extends SubsystemBase {
    private final AddressableLED leds = new AddressableLED(0);

    public LEDsSubsystem() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT);
        leds.setLength(buffer.getLength());
    }

    public void setBuffer(AddressableLEDBuffer buffer) {
         if (buffer.getLength() == LEDsConstants.LEDS_COUNT) {
             leds.setData(buffer);
         }
    }

    public void stop() {
         leds.stop();
    }

    public void start() {
         leds.start();
    }
}
