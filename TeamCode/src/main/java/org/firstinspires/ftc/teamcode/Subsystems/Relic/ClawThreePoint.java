package org.firstinspires.ftc.teamcode.Subsystems.Relic;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Sarthak on 11/8/2017.
 */

public class ClawThreePoint implements IRelic {
    private DcMotor relic_extension;
    private Servo arm, tilt, claw;
    private Telemetry telemetry;

    private double relicArmAngle = 0;
    private double relicTiltPos = 0;
    private double tiltOffset = 0.095;

    private final double RELIC_CLAW_CLOSED = .75;
    private final double RELIC_CLAW_OPENED = 0;

    private final double RELIC_TILT_ORIGIN = 0;

    private final double RELIC_ARM_ORIGIN = 0;

    public ClawThreePoint(DcMotor extension, Servo arm, Servo tilt, Servo claw, Telemetry telemetry){
        this.relic_extension = extension;
        this.arm = arm;
        this.tilt = tilt;
        this.claw = claw;
        this.telemetry = telemetry;
        this.claw.setPosition(RELIC_CLAW_CLOSED);
        this.arm.setPosition(RELIC_ARM_ORIGIN);
        this.tilt.setPosition(RELIC_TILT_ORIGIN);
        //this.relic_extension.setDirection(DcMotorSimple.Direction.REVERSE);
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relic_extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relic_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void pickUpRelic() {
        claw.setPosition(RELIC_CLAW_CLOSED);
    }

    @Override
    public void releaseRelic() {
        claw.setPosition(RELIC_CLAW_OPENED);
    }

    public void adjustArm(boolean condition, double increment){
        if(condition){
            arm.setPosition(arm.getPosition() + increment);
            if(arm.getPosition() > 0.685) {
                relicArmAngle = (arm.getPosition() - 0.685) / ((0.9) / 180);
                relicTiltPos = ((180-relicArmAngle) * (0.005)) + tiltOffset;
                tilt.setPosition(relicTiltPos);
            }
        }
    }

    public double returnArmAngle(){
        return relicArmAngle;
    }
    public double returnArmPos(){
        return arm.getPosition();
    }
    public double returnTiltPos(){
        return tilt.getPosition();
    }

    public void tiltRelic(boolean condition, double increment){
        if(condition){
            tilt.setPosition(tilt.getPosition() + increment);
        }
    }

    public void setArmPosition(double armPosition){
        double prevPosition = arm.getPosition();
        arm.setPosition(armPosition);
        if(arm.getPosition() > 0.74) {
            relicArmAngle = (arm.getPosition() - 0.74) / ((0.9 - .74) / 45);
            relicTiltPos = ((180-relicArmAngle) * (0.005)) + tiltOffset;
            tilt.setPosition(relicTiltPos);
        }else if(prevPosition > 0.74){
            relicArmAngle = (prevPosition - 0.74) / ((0.9 - .74) / 45);
            relicTiltPos = ((180-relicArmAngle) * (0.005)) + tiltOffset;
            tilt.setPosition(relicTiltPos);
        }
    }

    public void setTiltPosition(double tiltPosition){
        this.relicTiltPos = tiltPosition;
        tilt.setPosition(this.relicTiltPos);
    }

    @Override
    public void extend(double power, boolean condition) {
        if(condition) {
            relic_extension.setPower(power);
        }
    }

    @Override
    public void retract(double power, boolean condition) {
        if(condition) {
            relic_extension.setPower(power);
        }
    }

    public void extensionPowerZero(){
        relic_extension.setPower(0);
    }
}
