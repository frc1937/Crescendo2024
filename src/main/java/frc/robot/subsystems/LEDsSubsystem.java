package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LEDsConstants.LEDS_COUNT;

public class LEDsSubsystem extends SubsystemBase {
    private final AddressableLED leds = new AddressableLED(0);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDS_COUNT);

    private int counter;
    private int previousColour = 0;

    private int rainbowFirstPixel;
    private LEDState currentState;

    public LEDsSubsystem() {
        leds.setLength(LEDS_COUNT);
        leds.setData(buffer);
        leds.start();
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case SHOOTER_LOADED -> setBufferToBreathe(new Color8Bit(Color.kGreen), new Color8Bit(Color.kFloralWhite), Timer.getFPGATimestamp());
            case SHOOTER_EMPTY -> setBufferToCircling(new Color8Bit(Color.kDarkRed), new Color8Bit(Color.kRed));
            case BATTERY_LOW -> setBufferToFlashing(new Color8Bit(Color.kRed), new Color8Bit(Color.kWhite), new Color8Bit(Color.kGreen), new Color8Bit(1, 1, 1));
            case DEFAULT -> setBufferToRainbow();
        }

        leds.setData(buffer);
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

        for (var i = 0; i < LEDS_COUNT; i++) {
            hue = (rainbowFirstPixel + (i * 180 / buffer.getLength())) % 180;
            buffer.setHSV(i, hue, 255, 128);
        }

        rainbowFirstPixel += 3;
        rainbowFirstPixel %= 180;
    }

    private void setBufferToWholeColour(Color8Bit colour) {
        for (int i = 0; i < LEDS_COUNT; i++) {
            buffer.setLED(i, colour);
        }
    }

    //Switches between colours quickly.
    private void setBufferToFlashing(Color8Bit... colours) {
        if (counter % 25 == 0) //Make sure there's a delay between colour switching
            setBufferToWholeColour(colours[previousColour++]);

        previousColour %= colours.length;
        counter++;
    }

    //Slowly switch between two colours
    private void setBufferToBreathe(Color8Bit c1, Color8Bit c2, double timestamp) {
        double x = ((timestamp) * 2.0 * Math.PI);
        double ratio = (Math.sin(x) + 1.0) / 2.0;

        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);

        setBufferToWholeColour(new Color8Bit((int) red, (int) green, (int) blue));
    }

    //todo: make this support infinite colours
    private void setBufferToCircling(Color8Bit c1, Color8Bit c2) {
        int halfLength = LEDS_COUNT / 2;

        int startC1 = wrapIndex((int) (Timer.getFPGATimestamp() * 46 % LEDS_COUNT));
        int endC1 = startC1 + (halfLength - 1);

        int startC2 = endC1 + 1;
        int endC2 = startC2 + (halfLength - 1);

        for (int i = startC1; i < endC1; i++) {
            buffer.setLED(wrapIndex(i), c1);
        }

        for (int i = startC2; i < endC2; i++) {
            buffer.setLED(wrapIndex(i), c2);
        }
    }

    private int wrapIndex(int i) {
        while (i >= LEDS_COUNT)
            i -= LEDS_COUNT;
        return i;
    }
}
