package org.firstinspires.ftc.teamcode;

/**
 * Created by Sarthak on 9/24/2017.
 */

public interface IRelic {
    //Gains possession of relic
    void pickUpRelic();
    //Drop relic
    void releaseRelic();

    /**
     * @param power power to extend relic at
     */
    void extend(double power);

    /**
     * retract relic mechanism
     * @param power power to retract mechanism at
     */
    void retract(double power);
}
