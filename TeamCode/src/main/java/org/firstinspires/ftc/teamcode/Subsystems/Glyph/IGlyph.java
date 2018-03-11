package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

/**
 * Created by Sarthak on 9/24/2017.
 */

public interface IGlyph {
    //Intake and gain possession glyph
    void secureGlyph();

    //Release glyph into cryptobox
    void dispenseGlyph();

    //Turn off the intake mechanism
    void turnOff();

}
