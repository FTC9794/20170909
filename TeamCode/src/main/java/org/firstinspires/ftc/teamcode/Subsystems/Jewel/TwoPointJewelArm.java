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
    //hardware and information used by the jewel object
    Servo panServo, tiltServo;
    IColorSensor color;
    Telemetry telemetry;
    Alliance ballColor;

    //constants for the position at which the jewel pan should be when the jewel is knocked off either left or right
    final double POSITION_RIGHT_KNOCK = .7;
    final double POSITION_LEFT_KNOCK = .3;

    /**
     * Constructor for jewel arm with two degrees of freedom
     * @param ServoP Servo to pan the mechanism horizontally
     * @param ServoT Servo to tilt the mechanism vertically
     * @param color Color sensor to detect ball color
     * @param telemetry to display sensor values and debugging information
     */
    public TwoPointJewelArm(Servo ServoP, Servo ServoT, IColorSensor color, Telemetry telemetry){

        //set parameters of constructor to hardware in object
        this.panServo = ServoP;
        this.tiltServo = ServoT;
        this.color = color;
        this.telemetry = telemetry;
        this.ballColor = UNKNOWN;
    }

    /**
     * Reads the color of the ball
     * @param readings How many times the ball will be read
     * @return String of the color seen
     */
    @Override
    public String readColor(int readings) {

        //initialize variables to count the amount of times red has been seen vs. blue
        int blueCount = 0, redCount = 0;

        //repeat the readings
        for(int i = 0; i < readings; i++){

            //get an HSV reading of the ball
            float hsv = color.getHSV()[0];

            //determine if it is red or blue
            if(hsv >= 340 || hsv <= 30){
                redCount++;
            }else if (hsv >= 150 || hsv <= 225){
                blueCount++;
            }else{
            }
        }

        //compare the red counts vs. the blue counts to determine whether a blue or red ball is being seen
        if(blueCount > redCount){
            this.ballColor = BLUE;
            return "blue";
        }else if (redCount > blueCount){
            this.ballColor = RED;
            return "red";
        }else{
            this.ballColor = UNKNOWN;
            return "unknown";
        }
    }

    /**
     * Knocks off the jewel of the opposing alliance
     * @param alliance The alliance color of the robot
     * @param isLeftFast Whether to knock the jewel off fast if it needs to knock off the left
     * @param isRightFast Whether to knock the jewel off fast if it needs to knock off the right
     * @return Return true when method is complete or false if it isn't complete
     */
    @Override
    public boolean knockOffJewel(Alliance alliance, boolean isLeftFast, boolean isRightFast) {
        //determine which alliance the robot is on
        if (alliance == BLUE) {

            //if blue alliance check if blue ball
            if (ballColor == BLUE) {

                //determine if servo has moved enough
                if (this.panServo.getPosition() < POSITION_RIGHT_KNOCK) {

                    //move servo at different speed depending on parameter
                    if (isRightFast) {
                        this.setPanTiltPos(this.panServo.getPosition() + 0.005, 0.22);
                    } else {
                        this.setPanTiltPos(this.panServo.getPosition() + 0.001, 0.22);
                    }

                    //return false since the servo is not at the desired position yet
                    return false;
                } else {
                    //return true when the servo is at the desired position
                    return true;
                }

            //if blue alliance and ball is red
            } else if (ballColor == RED) {
                //check servo to see if at desired position
                if (panServo.getPosition() > POSITION_LEFT_KNOCK) {

                    //move either fast or slow depending on parameters
                    if (isLeftFast) {
                        this.setPanTiltPos(this.panServo.getPosition() - 0.005, 0.22);
                    } else {
                        this.setPanTiltPos(this.panServo.getPosition() - 0.001, 0.22);
                    }

                    //return false since servo is not at the desired position yet
                    return false;
                } else {
                    //return true when the servo is at the desired position
                    return true;
                }
            }else{
                //return true because ball color must be unknown
                return true;
            }

        //if alliance is red
        } else if (alliance == RED) {
            //if red alliance check if red ball
            if (ballColor == RED) {

                //check servo to see if at desired position
                if (this.panServo.getPosition() < POSITION_RIGHT_KNOCK) {

                    //move either fast or slow depending on parameters
                    if (isRightFast) {
                        this.setPanTiltPos(this.panServo.getPosition() + 0.005, 0.22);
                    } else {
                        this.setPanTiltPos(this.panServo.getPosition() + 0.001, 0.22);
                    }
                    return false;
                } else {
                    return true;
                }

            //Check to see if ball read was blue so need to knock off ball read
            } else if (ballColor == BLUE) {
                //check servo to see if at desired position
                if (panServo.getPosition() > POSITION_LEFT_KNOCK) {

                    //move either fast or slow depending on parameters
                    if (isLeftFast) {
                        this.setPanTiltPos(this.panServo.getPosition() - 0.005, 0.22);
                    } else {
                        this.setPanTiltPos(this.panServo.getPosition() - 0.001, 0.22);
                    }
                    return false;
                } else {
                    return true;
                }
            }else{
                //return true to stop method because ball color must be unknown
                return true;
            }
        } else {
            //return true because alliance must be null
            return true;
        }

    }

    /**
     * Sets the position of the tilt servo
     * @param tiltPosition set the tilt position of the jewel tilt servo
     */
    private void tilt(double tiltPosition){
        tiltServo.setPosition(tiltPosition);
    }

    /**
     * Sets the position of the pan servo
     * @param panPosition set the pan position of the jewel pan servo
     */
    private void pan(double panPosition) { panServo.setPosition(panPosition); }

    /**
     * Sets the position of the pan servo and the tilt servo
     * @param panPosition The position at which to move the pan servo
     * @param tiltPosition The position at which to move the tilt servo
     * @return Return true since method is complete as soon as it is run
     */
    public boolean setPanTiltPos(double panPosition, double tiltPosition){
        pan(panPosition);
        tilt(tiltPosition);
        return true;
    }
}
