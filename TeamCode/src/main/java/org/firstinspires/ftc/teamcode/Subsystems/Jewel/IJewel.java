package org.firstinspires.ftc.teamcode.Subsystems.Jewel;

import org.firstinspires.ftc.teamcode.Enums.Alliance;

/**
 * Created by Sarthak on 9/24/2017.
 */

public interface IJewel {
    /**
     * Read the color of the ball
     * @param readings the number of readings the sensor should collect
     * @return the color of the ball as a string
     */
    String readColor(int readings);

    /**
     * Knock off the jewel, based on the reading of the ball and the alliance color
     * @param alliance the alliance color the robot is on
     * @param isLeftFast if the left ball is being knocked off, should it be done fast or slow
     * @param isRightFast if the right ball is being knocked off, should it be done fast or slow
     * @return true if the action has been completed, false if the action is ongoing
     */
    boolean knockOffJewel(Alliance alliance, boolean isLeftFast, boolean isRightFast);
}
