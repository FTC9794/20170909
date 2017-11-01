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
    String ballColor;

    public TwoPointJewelArm(Servo ServoP, Servo ServoT, IColorSensor color, Telemetry telemetry){
        this.panServo = ServoP;
        this.tiltServo = ServoT;
        this.color = color;
        this.telemetry = telemetry;
        this.ballColor = "";
    }

    @Override
    public String readColor(int readings) {
        int blueCount = 0, redCount = 0;
        int red = color.red();
        int blue = color.blue();
        telemetry.addData("Blue", blue);
        telemetry.addData("Red", red);
        for(int i = 0; i < readings; i++){
            if(blue > red*1.5){
                blueCount++;
                telemetry.addData("Color", "blue");
            }else if (red > blue*1.5){
                redCount++;
                telemetry.addData("Color", "Red");
            }else{
                telemetry.addData("Color", "Unknown");
            }
        }
        if(blueCount > redCount){
            telemetry.addData("Color", "blue");
            this.ballColor = "blue";
            return "blue";
        }else if (redCount > blueCount){
            telemetry.addData("Color", "Red");
            this.ballColor = "red";
            return "red";
        }else{
            telemetry.addData("Color", "Unknown");
            this.ballColor = "unknown";
            return "unknown";
        }
    }

    @Override
    public boolean knockOffJewel(String alliance) {
        panServo.setPosition(0.87);
        tiltServo.setPosition(0.03);
        if(alliance.equals("blue")){
            if(ballColor.equals("blue")){
                panServo.setPosition(0.8);
            }else{
                panServo.setPosition(0.98);
            }
        }else if(alliance.equals("red")){
            if(ballColor.equals("red")){
                panServo.setPosition(0.8);
            }else{
                panServo.setPosition(0.98);
            }
        }
        return true;
    }

    private void tilt(double tiltPosition){
        tiltServo.setPosition(tiltPosition);
    }

    private void pan(double panPosition) { panServo.setPosition(panPosition); }

    public boolean setPanTiltPos(double panPosition, double tiltPosition){
        pan(panPosition);
        tilt(tiltPosition);
        return true;
    }
}
