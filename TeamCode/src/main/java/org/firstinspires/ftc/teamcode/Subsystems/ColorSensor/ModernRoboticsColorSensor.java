package org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Sarthak on 10/16/2017.
 */

public class ModernRoboticsColorSensor implements IColorSensor {
    ColorSensor color;

    public ModernRoboticsColorSensor(ColorSensor color){
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

    @Override
    public float[] getHSV() {
        float[] hsv = {0F, 0F, 0F};
        Color.RGBToHSV(this.red(), this.green(), this.blue(), hsv);
        return hsv;
    }
}
