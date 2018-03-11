package org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;

/**
 * Created by Sarthak on 10/15/2017.
 */

public interface IColorSensor {
    //Gets the red value from the sensor
    public int red();
    //Gets the green value from the sensor
    public int green();
    //Gets the blue value from the sensor
    public int blue();
    //Gets the hue from the sensor
    public int getHue();
    //Gets the alpha reading from the sensor
    public int alpha();
    //Gets the HSV values from the sensor and return the three values as a float array
    public float[] getHSV();
}
