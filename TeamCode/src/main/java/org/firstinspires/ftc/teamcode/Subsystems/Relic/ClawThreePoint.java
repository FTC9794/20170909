package org.firstinspires.ftc.teamcode.Subsystems.Relic;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Sarthak on 11/8/2017.
 */

public class ClawThreePoint implements IRelic {
    //Create motor for relic extension
    private DcMotor relic_extension;
    //Create servos for controlling relic arm
    private Servo arm, tilt, claw;

    //Variables for servo positions and calculations
    private double relicArmAngle = 0;
    private double relicTiltPos = 0;
    private double tiltOffset = -0.02;

    //Create constants to set servo values
    private final double RELIC_CLAW_CLOSED = 1;
    private final double RELIC_CLAW_OPENED = 0;
    private final double RELIC_TILT_ORIGIN = 0;
    private final double RELIC_ARM_ORIGIN = 0;

    /**
     * Constructor for Relic Mechanism with three degrees of freedom
     * @param extension motor to extend slides
     * @param arm servo to control arm action
     * @param tilt servo to tilt the claw
     * @param claw servo to open and close the claw
     */
    public ClawThreePoint(DcMotor extension, Servo arm, Servo tilt, Servo claw){
        //Set values to instance variables
        this.relic_extension = extension;
        this.arm = arm;
        this.tilt = tilt;
        this.claw = claw;
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relic_extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relic_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set the relic claw to a position to pick up the relic
     */
    @Override
    public void pickUpRelic() {
        claw.setPosition(RELIC_CLAW_CLOSED);
    }

    /**
     * Set the relic claw to a position to release the relic
     */
    @Override
    public void releaseRelic() {
        claw.setPosition(RELIC_CLAW_OPENED);
    }

    /**
     * Adjust the position of the servo arm while keeping the tilt servo perpendicular to the ground
     * @param condition the condition that allows the servo arm to move
     * @param increment the increment that the servo arm position will change by
     */
    public void adjustArm(boolean condition, double increment){
        if(condition){
            //Set the position of the arm
            arm.setPosition(arm.getPosition() + increment);
            //Calculate servo positions and set the tilt servo to be perpendicular to the ground
            if(arm.getPosition() > 0.685) {
                relicArmAngle = (arm.getPosition() - 0.685) / ((0.9) / 180);
                relicTiltPos = ((180-relicArmAngle) * (0.005)) + tiltOffset;
                tilt.setPosition(relicTiltPos);
            }
        }
    }

    /**
     * Return the position of the relic arm servo
     * @return the servo position as a double
     */
    public double returnArmPos(){
        return arm.getPosition();
    }

    /**
     * Return th position of the relic tilt servo
     * @return the servo position as a double
     */
    public double returnTiltPos(){
        return tilt.getPosition();
    }

    /**
     * Adjust the position of the relic tilt servo
     * @param condition the condition at which the servo position would adjust
     * @param increment the increment at which the servo position will adjust
     */
    public void tiltRelic(boolean condition, double increment){
        if(condition){
            tilt.setPosition(tilt.getPosition() + increment);
        }
    }

    /**
     * Set the relic arm position while automatically adjusting the tilt servo to keep the claw perpendicular the to ground
     * @param armPosition the position to set the servo, ranging from 0.0 to 1.0
     */
    public void setArmPosition(double armPosition){
        double prevPosition = arm.getPosition();
        arm.setPosition(armPosition);
        if(arm.getPosition() > 0.7) {
            relicArmAngle = (arm.getPosition() - 0.7) / ((.7) / 180);
            relicTiltPos = ((180-relicArmAngle) * (0.005)) + tiltOffset;
            tilt.setPosition(relicTiltPos);
        }else if(prevPosition > 0.7){
            relicArmAngle = (prevPosition - 0.7) / ((.7) / 180);
            relicTiltPos = ((180-relicArmAngle) * (0.005)) + tiltOffset;
            tilt.setPosition(relicTiltPos);
        }
    }

    /**
     * Set the position of the tilt servo
     * @param tiltPosition the position to set the servo, ranging from 0.0 to 1.0
     */
    public void setTiltPosition(double tiltPosition){
        this.relicTiltPos = tiltPosition;
        tilt.setPosition(this.relicTiltPos);
    }

    /**
     * Extend the relic slides
     * @param power power to extend relic at, ranging from 0 to 1.0
     * @param condition the condition at which the relic slides will extend
     */
    @Override
    public void extend(double power, boolean condition) {
        if(condition) {
            relic_extension.setPower(power);
        }
    }

    /**
     * Retract the relic slides
     * @param power power to retract mechanism at, ranging from -1.0 to 0
     * @param condition the condition at which the relic slides will retract
     */
    @Override
    public void retract(double power, boolean condition) {
        if(condition) {
            relic_extension.setPower(power);
        }
    }

    /**
     * Set the power of the relic extension motor to 0
     */
    public void extensionPowerZero(){
        relic_extension.setPower(0);
    }

    public void resetRelicEncoder(){
        relic_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
