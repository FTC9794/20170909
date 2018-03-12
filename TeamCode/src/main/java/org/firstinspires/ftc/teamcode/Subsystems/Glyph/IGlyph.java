package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

/**
 * Created by Sarthak on 9/24/2017.
 */

public interface IGlyph {
    /**
     * Bring glyph into robot's possession
     */
    void secureGlyph();

    /**
     * Dispense the glyph from the robot
     */
    void dispenseGlyph();

    /**
     * Turn off the glyph intake mechanism (set motor powers to zero or servos to default position)
     */
    void turnOff();

}
