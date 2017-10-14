package org.firstinspires.ftc.teamcode.Subsystems.Jewel;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sarthak on 10/12/2017.
 */

public class TwoPointJewelArm implements IJewel {
    Servo panServo, tiltServo;
    ColorSensor color;

    public TwoPointJewelArm(Servo ServoP, Servo ServoT, ColorSensor color){
        this.panServo = ServoP;
        this.tiltServo = ServoT;
        this.color = color;
    }

    @Override
    public String readColor() {
        int red = color.red();
        int blue = color.blue();
        if(red > blue){
            return "red";
        }else{
            return "blue";
        }
    }

    @Override
    public void knockOffJewel(String alliance) {
        if(alliance.equals("blue")){

        }else if(alliance.equals("red")){

        }
    }

    public void tilt(double tiltPosition){
        tiltServo.setPosition(tiltPosition);
    }
}
