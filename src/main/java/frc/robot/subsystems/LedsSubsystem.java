package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedsSubsystem extends SubsystemBase {
    private final AddressableLED led = new AddressableLED(0);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(105);
    private String currentAnimation = "";
    private int rainbowFirstPixelHue = 0;

    public LedsSubsystem() {
        led.setLength(ledBuffer.getLength());
        led.setSyncTime(400);
        led.start();
    }

    @Override
    public void periodic() {
        if (currentAnimation.equals("rainbow")) {
            rainbow();
        }
    }

    public void startLEDs(String animation) {
        if ("blue".equals(animation)) {
            animateBlueLight();
            currentAnimation = "blue";
        }

        if ("red".equals(animation)) {
            animateRedLight();
            currentAnimation = "red";
        }

        if("green".equals("animation")) {
            animateGreenLight();
            currentAnimation = "green";
        }

        if("rainbow".equals(animation)) {
            currentAnimation = "rainbow";
            rainbow();
        }

        led.setData(ledBuffer);
    }

    private void rainbow() {
        int hue;

        for (var i = 0; i < ledBuffer.getLength(); i++) {
            hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, 255, 128);
        }

        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }


    private void animateBlueLight() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 122);
        }
    }

    private void animateGreenLight() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 122, 0);
        }
    }

    private void animateRedLight() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 122, 0, 0);
        }
    }
}
