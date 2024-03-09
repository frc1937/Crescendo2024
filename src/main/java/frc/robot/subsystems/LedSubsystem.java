package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED led = new AddressableLED(0);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(30);

    public LedSubsystem() {
        led.setLength(buffer.getLength());
        led.start();
    }

    public void setLedColour(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }

        led.setData(buffer);
    }
}
