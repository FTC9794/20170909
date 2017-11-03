package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sarthak on 10/28/2017.
 */

public class FourArmRotatingGlyph implements  IGlyph {

    private Servo right1,right2,left1,left2,spin;
    private DcMotor lift;

    private final double GRIP_OPEN1 = .5;
    private final double GRIP_OPEN2 = .5;
    private final double GRIP_CLOSE1 = 0;
    private final double GRIP_CLOSE2 = 0;


    final double SPIN_START = 0;
    final double SPIN_ROTATED = 1;

    public FourArmRotatingGlyph(Servo right1, Servo right2, Servo left1, Servo left2, Servo spin, DcMotor lift){
        this.right1 = right1;
        this.right2 = right2;
        this.left1 = left1;
        this.left2 = left2;

        this.spin = spin;
        this.lift = lift;
    }

    @Override
    public void secureGlyph() {
        if(this.spin.getPosition() == 0){
            this.right1.setPosition(GRIP_CLOSE1);
            this.left1.setPosition(GRIP_CLOSE1);
        }else if(spin.getPosition() == 1){
            this.left2.setPosition(GRIP_CLOSE2);
            this.right2.setPosition(GRIP_CLOSE2);
        }
    }

    @Override
    public void dispenseGlyph() {
        if(this.spin.getPosition() == 0){
            this.left1.setPosition(GRIP_OPEN1);
            this.right1.setPosition(GRIP_CLOSE1);
        }else if(spin.getPosition() == 1){
            this.left2.setPosition(GRIP_OPEN2);
            this.right2.setPosition(GRIP_OPEN2);
        }
    }

    public void gripAll(){
        this.left1.setPosition(GRIP_CLOSE1);
        this.right1.setPosition(GRIP_CLOSE1);
        this.left2.setPosition(GRIP_CLOSE2);
        this.right2.setPosition(GRIP_CLOSE2);
    }

    public void dispenseAll(){
        this.left1.setPosition(GRIP_OPEN1);
        this.right1.setPosition(GRIP_OPEN1);
        this.left2.setPosition(GRIP_OPEN2);
        this.right2.setPosition(GRIP_OPEN2);
    }

    public void rotate(){
        if(this.spin.getPosition() == 0){
            this.spin.setPosition(1);
        }else if (this.spin.getPosition() == 1){
            this.spin.setPosition(0);
        }
    }

    @Override
    public void changeHeight(double power, boolean condition) {
        if(condition){
            this.lift.setPower(power);
        }else{
            this.lift.setPower(0);
        }
    }
}
