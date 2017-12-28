package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sarthak on 10/28/2017.
 */

public class FourArmRotatingGlyph implements  IGlyph {

    private Servo right1, right2, left1, left2, spin;
    private DcMotor lift;
    private DigitalChannel glyphLimit;

    private boolean topGripOpen = true;
    private boolean bottomGripOpen = true;
    private boolean spinAtOrigin = true;
    private boolean spinPressed = false;
    private boolean gripPressed1 = false;
    private boolean gripPressed2 = false;
    private boolean rightBumperPressed = false;
    private boolean leftBumperPressed = false;
    private boolean lowerLift = false;
    private boolean resetPressed = false;

    final double GRIP_OPEN1 = .5;
    final double GRIP_OPEN2 = .5;
    final double GRIP_CLOSE1 = 0;
    final double GRIP_CLOSE2 = 0;
    final double SPIN_START = 0;
    final double SPIN_ROTATED = .95;

    private enum SPIN_STATE{
        ORIGIN,
        FLIPPED
    }

    public FourArmRotatingGlyph(Servo right1, Servo right2, Servo left1, Servo left2, Servo spin, DcMotor lift){
        this.right1 = right1;
        this.right2 = right2;
        this.left1 = left1;
        this.left2 = left2;

        this.spin = spin;
        this.lift = lift;
    }

    public void initialize(){
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right2.setDirection(Servo.Direction.REVERSE);
        left1.setDirection(Servo.Direction.REVERSE);

        right1.setPosition(GRIP_OPEN1);
        right2.setPosition(GRIP_OPEN2);
        left1.setPosition(GRIP_OPEN1);
        left2.setPosition(GRIP_OPEN2);

        spin.setPosition(SPIN_START);
    }

    @Override
    public boolean secureGlyph() {
        if(spinAtOrigin){
            right2.setPosition(GRIP_CLOSE2);
            left2.setPosition(GRIP_CLOSE2);
        }else{
            right1.setPosition(GRIP_CLOSE1);
            left1.setPosition(GRIP_CLOSE1);
        }
        return true;
    }

    public boolean secureTopGlyph() {
        if (spinAtOrigin) {
            right1.setPosition(GRIP_CLOSE1);
            left1.setPosition(GRIP_CLOSE1);
        } else {
            right2.setPosition(GRIP_CLOSE2);
            left2.setPosition(GRIP_CLOSE2);
        }
        return true;
    }

    @Override
    public boolean dispenseGlyph() {
        if(spinAtOrigin){
            right2.setPosition(GRIP_OPEN2);
            left2.setPosition(GRIP_OPEN2);
        }else{
            right1.setPosition(GRIP_OPEN1);
            left1.setPosition(GRIP_OPEN1);
        }
        return true;
    }

    public void dispenseTopGlyph(){
        if (spinAtOrigin) {
            right1.setPosition(GRIP_OPEN1);
            left1.setPosition(GRIP_OPEN1);
        } else {
            right2.setPosition(GRIP_OPEN2);
            left2.setPosition(GRIP_OPEN2);
        }
    }

    public void rotate(){
        if(spinAtOrigin) {
            spin.setPosition(SPIN_ROTATED);
            spinAtOrigin = false;
        }else{
            spin.setPosition(SPIN_START);
            spinAtOrigin = true;
        }
    }

    @Override
    public boolean changeHeight(double power, boolean condition) {
        if(condition){
            this.lift.setPower(power);
            return true;
        }else{
            this.lift.setPower(0);
            return false;
        }
    }
}
