package org.firstinspires.ftc.teamcode.Subsystems.Jewel;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;

/**
 * Created by Sarthak on 10/12/2017.
 */

public class TwoPointJewelArm implements IJewel {
    Servo panServo, tiltServo;
    IColorSensor color;

    public TwoPointJewelArm(Servo ServoP, Servo ServoT, IColorSensor color){
        this.panServo = ServoP;
        this.tiltServo = ServoT;
        this.color = color;
    }

    @Override
    public String readColor() {
        int hue = color.getHue();
        if(hue >= 350  || hue <= 30){
            return "red";
        }else if (hue >= 150 && hue <= 255){
            return "blue";
        }else{
            return "unknown";
        }
    }

    @Override
    public void knockOffJewel(String alliance) {
        panServo.setPosition(0.87);
        tiltServo.setPosition(0.03);
        if(alliance.equals("blue")){
            if(alliance.equals("blue")){
                panServo.setPosition(0.8);
            }else{
                panServo.setPosition(0.98);
            }
        }else if(alliance.equals("red")){
            if(readColor().equals("red")){
                panServo.setPosition(0.8);
            }else{
                panServo.setPosition(0.98);
            }
        }
    }

    public void tilt(double tiltPosition){
        tiltServo.setPosition(tiltPosition);
    }

    public void pan(double panPosition) { panServo.setPosition(panPosition); }
}
