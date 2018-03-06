package org.firstinspires.ftc.teamcode.Subsystems.Jewel;

/**
 * Created by Sarthak on 9/24/2017.
 */

public interface IJewel {
    String readColor(int readings);     //Return the color of jewel(s)
    boolean knockOffJewel(String alliance, boolean isLeftFast, boolean isRightFast);
}
