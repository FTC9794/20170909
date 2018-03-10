package org.firstinspires.ftc.teamcode.Subsystems.Jewel;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DataLogger;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.Enums.Alliance.RED;
import static org.firstinspires.ftc.teamcode.Enums.Alliance.UNKNOWN;

/**
 * Created by Sarthak on 10/12/2017.
 */

public class TwoPointJewelArm implements IJewel {
    Servo panServo, tiltServo;
    IColorSensor color;
    Telemetry telemetry;
    Alliance ballColor;

    public TwoPointJewelArm(Servo ServoP, Servo ServoT, IColorSensor color, Telemetry telemetry){
        this.panServo = ServoP;
        this.tiltServo = ServoT;
        this.color = color;
        this.telemetry = telemetry;
        this.ballColor = UNKNOWN;
    }

    @Override
    public String readColor(int readings) {
        int blueCount = 0, redCount = 0;
        float hsv = color.getHSV()[0];
        //telemetry.addData("HSV", hsv);
        for(int i = 0; i < readings; i++){
            if(hsv >= 340 || hsv <= 30){
                redCount++;
            }else if (hsv >= 150 || hsv <= 225){
                blueCount++;
            }else{
            }
        }
        if(blueCount > redCount){
            //telemetry.addData("Color", "blue");
            this.ballColor = BLUE;
            return "blue";
        }else if (redCount > blueCount){
            //telemetry.addData("Color", "Red");
            this.ballColor = RED;
            return "red";
        }else{
            //telemetry.addData("Color", "Unknown");
            this.ballColor = UNKNOWN;
            return "unknown";
        }
    }

    @Override
    public boolean knockOffJewel(Alliance alliance, boolean isLeftFast, boolean isRightFast) {
        if (alliance == BLUE) {
            if (ballColor == BLUE) {
                if (this.panServo.getPosition() < 0.7) {
                    if (isRightFast) {
                        this.setPanTiltPos(this.panServo.getPosition() + 0.005, 0.22);
                    } else {
                        this.setPanTiltPos(this.panServo.getPosition() + 0.001, 0.22);
                    }
                    return false;
                } else {
                    return true;
                }
            } else if (ballColor == RED) {
                if (panServo.getPosition() > 0.3) {
                    if (isLeftFast) {
                        this.setPanTiltPos(this.panServo.getPosition() - 0.005, 0.22);
                    } else {
                        this.setPanTiltPos(this.panServo.getPosition() - 0.001, 0.22);
                    }
                    return false;
                } else {
                    return true;
                }
            }
        } else if (alliance == RED) {
            if (ballColor == RED) {
                if (this.panServo.getPosition() < 0.7) {
                    if (isRightFast) {
                        this.setPanTiltPos(this.panServo.getPosition() + 0.005, 0.22);
                    } else {
                        this.setPanTiltPos(this.panServo.getPosition() + 0.001, 0.22);
                    }
                    return false;
                } else {
                    return true;
                }
            } else if (ballColor == BLUE) {
                if (panServo.getPosition() > 0.3) {
                    if (isLeftFast) {
                        this.setPanTiltPos(this.panServo.getPosition() - 0.005, 0.22);
                    } else {
                        this.setPanTiltPos(this.panServo.getPosition() - 0.001, 0.22);
                    }
                    return false;
                } else {
                    return true;
                }
            }
        } else {
            return true;
        }
        return false;
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
