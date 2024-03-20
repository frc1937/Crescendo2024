package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDsConstants;

public class LEDsSubsystem extends SubsystemBase {
    private final AddressableLED leds = new AddressableLED(0);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT);

    public LEDsSubsystem() {
        leds.setLength(buffer.getLength());
        leds.start();
    }

    public void setBuffer(AddressableLEDBuffer buffer) {
        if (buffer.getLength() == LEDsConstants.LEDS_COUNT) {
            leds.setData(buffer);
        }
    }

    public void clear() {
        leds.stop();
    }
}
