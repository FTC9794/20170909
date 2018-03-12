package org.firstinspires.ftc.teamcode.Subsystems.Relic;

/**
 * Created by Sarthak on 9/24/2017.
 */

public interface IRelic {
    /**
     * Gain possession of the relic
     */
    void pickUpRelic();

    /**
     * Release the relic from the robot's possession
     */
    void releaseRelic();

    /**
     * Extend the relic mechanism
     * @param power power to extend relic at
     */
    void extend(double power, boolean condition);

    /**
     * retract relic mechanism
     * @param power power to retract mechanism at
     */
    void retract(double power, boolean condition);
}
