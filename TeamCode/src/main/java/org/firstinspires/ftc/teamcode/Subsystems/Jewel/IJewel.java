package org.firstinspires.ftc.teamcode.Subsystems.Jewel;

/**
 * Created by Sarthak on 9/24/2017.
 */

public interface IJewel {
    String readColor();     //Return the color of jewel(s)
    void knockOffLeftJewel();     //Knocks off the jewel on left
    void knockOffRightJewel();     //Knocks off the jewel on right
}
