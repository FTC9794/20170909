package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sarthak on 10/28/2017.
 */

public class FourArmRotatingGlyph implements  IGlyph {

    private Servo tl,tr,bl,br,spin;
    private DcMotor lift;

    public FourArmRotatingGlyph(Servo topLeft, Servo topRight, Servo bottomLeft, Servo bottomRight, Servo spin, DcMotor lift){
        this.tl = topLeft;
        this.tr = topRight;
        this.bl = bottomLeft;
        this.br = bottomRight;
        this.lift = lift;
    }

    @Override
    public void secureGlyph() {
    }

    @Override
    public void dispenseGlyph() {

    }

    public void rotate(){

    }

    @Override
    public void changeHeight(double power, boolean condition) {

    }
}
