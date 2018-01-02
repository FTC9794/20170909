package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by ishaa on 1/2/2018.
 */

public class twoWheelIntake implements IGlyph {
    private double intakePower = 1;
    private double outtakePower = -1;
    private CRServo left, right;
    public twoWheelIntake(CRServo leftIntake, CRServo rightIntake){
        left = leftIntake;
        right = rightIntake;
    }
    public twoWheelIntake(CRServo leftIntake, CRServo rightIntake, double intakePower, double outtakePower){
        left = leftIntake;
        right = rightIntake;
        this.intakePower = intakePower;
        this.outtakePower = outtakePower;
    }
    @Override
    public boolean secureGlyph() {
        left.setPower(intakePower);
        right.setPower(intakePower);
        return true;
    }

    @Override
    public boolean dispenseGlyph() {
        left.setPower(outtakePower);
        right.setPower(outtakePower);
        return true;
    }

    @Override
    public boolean changeHeight(double power, boolean condition) {
        return false;
    }

    @Override
    public boolean turnOff() {
        left.setPower(0);
        right.setPower(0);
        return true;
    }
}
