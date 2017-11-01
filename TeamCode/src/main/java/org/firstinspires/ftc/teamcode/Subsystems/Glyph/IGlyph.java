package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

/**
 * Created by Sarthak on 9/24/2017.
 */

public interface IGlyph {
    //Intake and gain possession glyph
    void secureGlyph();

    //Release glyph into cryptobox
    void dispenseGlyph();

    /**
     * Adjust the height of the glyph
     * @param power The power the glyph handler will move at to adjust position
     * @param condition When the glyph handler will stop moving
     */
    void changeHeight(double power, boolean condition);

}