package org.firstinspires.ftc.teamcode.Subsystems.Jewel;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DataLogger;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;

/**
 * Created by Sarthak on 10/12/2017.
 */

public class TwoPointJewelArm implements IJewel {
    Servo panServo, tiltServo;
    IColorSensor color;
    Telemetry telemetry;

    public TwoPointJewelArm(Servo ServoP, Servo ServoT, IColorSensor color, Telemetry telemetry){
        this.panServo = ServoP;
        this.tiltServo = ServoT;
        this.color = color;
        this.telemetry = telemetry;
    }

    @Override
    public String readColor() {
        int red = color.red();
        int blue = color.blue();
        telemetry.addData("Blue", blue);
        telemetry.addData("Red", red);
        if(blue > red*1.5){
            telemetry.addData("Color", "blue");
            return "blue";
        }else if (red > blue*1.5){
            telemetry.addData("Color", "Red");
            return "red";
        }else{
            telemetry.addData("Color", "Unknown");
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

    public void setPanTiltPos(double panPosition, double tiltPosition){
        pan(panPosition);
        tilt(tiltPosition);
    }
}
