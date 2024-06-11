package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDsConstants;

public class LEDsSubsystem extends SubsystemBase {
    private final AddressableLED leds = new AddressableLED(0);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDsConstants.LEDS_COUNT);

    private int counter;
    private int previousColour = 0;

    private int rainbowFirstPixel;
    private LEDState currentState;

    public LEDsSubsystem() {
        leds.setLength(LEDsConstants.LEDS_COUNT);
        leds.setData(buffer);
        leds.start();
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case SHOOTER_LOADED -> setBufferToStripes(new Color8Bit(Color.kOrange), new Color8Bit(Color.kRed));
            case SHOOTER_EMPTY -> setBufferToWholeColour(new Color8Bit(Color.kBlue));
            case BATTERY_LOW -> setBufferToFlashing(new Color8Bit(Color.kRed), new Color8Bit(Color.kWhite));
            case DEFAULT -> setBufferToRainbow();
        }

        leds.setData(buffer);
    }

    public void setBuffer(AddressableLEDBuffer buffer) {
         if (buffer.getLength() == LEDsConstants.LEDS_COUNT) {
             leds.setData(buffer);
         }
    }

    public void setLEDsState(LEDState ledState) {
         currentState = ledState;
    }

    public enum LEDState {
        SHOOTER_LOADED,
        SHOOTER_EMPTY,
        BATTERY_LOW,
        DEFAULT
    }

    private void setBufferToRainbow() {
        int hue;

        for (var i = 0; i < LEDsConstants.LEDS_COUNT; i++) {
            hue = (rainbowFirstPixel + (i * 180 / buffer.getLength())) % 180;
            buffer.setHSV(i, hue, 255, 128);
        }

        rainbowFirstPixel += 3;
        rainbowFirstPixel %= 180;
    }

    private void setBufferToWholeColour(Color8Bit colour) {
        for (int i = 0; i < LEDsConstants.LEDS_COUNT; i++) {
            buffer.setLED(i, colour);
        }
    }

    private void setBufferToFlashing(Color8Bit... colours) {
        if(counter % 50 == 0) //Make sure there's a delay between colour switching
            setBufferToWholeColour(colours[previousColour++]);

        previousColour %= colours.length;
        counter++;
    }

    private void setBufferToStripes(Color8Bit... colours) {
        int colourIndex;

        for (int i = 0; i < LEDsConstants.LEDS_COUNT; i++) {
            colourIndex = (int) (Math.floor((double) (i) / LEDsConstants.LEDS_COUNT) + colours.length) % colours.length;
            colourIndex = (colours.length - 1) - colourIndex;

            buffer.setLED(i, colours[colourIndex]);
        }
    }
}
