package org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;

/**
 * Created by Sarthak on 10/15/2017.
 */

public class LynxColorRangeSensor implements IColorSensor {

    LynxI2cColorRangeSensor color;

    public LynxColorRangeSensor(LynxI2cColorRangeSensor color){
        this.color = color;
    }

    @Override
    public int red() {
        return color.red();
    }

    @Override
    public int green() {
        return color.green();
    }

    @Override
    public int blue() {
        return color.blue();
    }

    @Override
    public int getHue() {
        return color.argb();
    }

    @Override
    public int alpha() {
        return color.alpha();
    }
}
