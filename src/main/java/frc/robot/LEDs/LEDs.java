package frc.robot.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase{
    public static class Settings {
        static final int kLEDId = 0;
        static final int kLEDBuffer = 16;
    }

    public static LEDs mInstance;
    public static AddressableLED mLED; 
    public static AddressableLEDBuffer mLEDBuffer;
    public static int mRainbowFirstPixelHue = 0;

    private LEDs() {
        mLED = new AddressableLED(Settings.kLEDId);
        mLEDBuffer = new AddressableLEDBuffer(Settings.kLEDBuffer);

        mLED.setLength(mLEDBuffer.getLength());
        setLEDColor(255, 60, 0);
        mLED.setData(mLEDBuffer);
        mLED.start();
    }

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    public void rainbowMode() {
        // For every pixel
        for (var i = 0; i < mLEDBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final int hue = (mRainbowFirstPixelHue + (i * 180 / mLEDBuffer.getLength())) % 180;
            // Set the value
            mLEDBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        mRainbowFirstPixelHue += 3;
        // Check bounds
        mRainbowFirstPixelHue %= 180;
        mLED.setData(mLEDBuffer);
  
    }

    public void setLEDColor(int red, int green, int blue) {
        for (var i = 0; i < mLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the HSV values for red
            mLEDBuffer.setRGB(i, red, green, blue);
        }
        mLED.setData(mLEDBuffer);
    }

    @Override
    public void periodic() {

    }
}
