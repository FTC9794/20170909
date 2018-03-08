package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Sarthak on 11/11/2017.
 */

public class DualWheelIntake implements IGlyph {
    private CRServo rightWheel1, leftWheel1, rightWheel2, leftWheel2;
    private Servo spin;
    private DcMotor lift;
    private DigitalChannel glyphLimit;

    Telemetry telemetry;

    private final double INTAKE_SPPED = 0.74;
    private final double OUTTAKE_SPEED = -0.74;
    final double SPIN_START = 0.825;
    final double SPIN_ROTATED = 0;

    boolean spinAtOrigin = true;

    public DualWheelIntake(CRServo rightWheel1, CRServo rightWheel2, CRServo leftWheel1, CRServo leftWheel2, Servo spin, DcMotor lift, DigitalChannel glyphLimit, Telemetry telemetry){
        this.rightWheel1 = rightWheel1;
        this.rightWheel2 = rightWheel2;
        this.leftWheel1 = leftWheel1;
        this.leftWheel2 = leftWheel2;


        this.spin = spin;
        this.telemetry = telemetry;
        this.lift = lift;
        this.glyphLimit = glyphLimit;

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        spin.setPosition(SPIN_START);
    }

    @Override
    public void secureGlyph() {
        rightWheel1.setPower(INTAKE_SPPED);
        rightWheel2.setPower(INTAKE_SPPED);
        leftWheel1.setPower(INTAKE_SPPED);
        leftWheel2.setPower(INTAKE_SPPED);

    }
    public boolean secureGlyph1(){
        rightWheel1.setPower(INTAKE_SPPED);
        leftWheel1.setPower(INTAKE_SPPED);
        return true;
    }
    public boolean stopGlyph1(){
        rightWheel1.setPower(0);
        leftWheel1.setPower(0);
        return true;
    }
    public boolean dispenseGlyph1(){
        rightWheel1.setPower(OUTTAKE_SPEED);
        leftWheel1.setPower(OUTTAKE_SPEED);
        return true;
    }
    @Override
    public void dispenseGlyph() {
        rightWheel1.setPower(OUTTAKE_SPEED);
        rightWheel2.setPower(OUTTAKE_SPEED);
        leftWheel1.setPower(OUTTAKE_SPEED);
        leftWheel2.setPower(OUTTAKE_SPEED);

    }

    public boolean setIntakePower(double leftPower, double rightPower){
        rightWheel1.setPower(rightPower);
        rightWheel2.setPower(rightPower);
        leftWheel1.setPower(leftPower);
        leftWheel2.setPower(leftPower);
        return true;
    }

    public boolean setIntakePowerZero(){
        rightWheel1.setPower(0);
        rightWheel2.setPower(0);
        leftWheel1.setPower(0);
        leftWheel2.setPower(0);
        return true;
    }


    public boolean changeHeight(double power, boolean condition) {
        if(condition){
            lift.setPower(power);
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void turnOff() {

    }

    public boolean setLiftPower(double power){
        lift.setPower(power);
        return true;
    }

    public boolean setLiftPowerZero(boolean condition){
        if(condition){
            lift.setPower(0);
            return true;
        }else{
            return false;
        }
    }

    public boolean checkGlyphLiftLimit(){
        if (!glyphLimit.getState()) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return true;
        }else{
            return false;
        }
    }

    public boolean setLiftTargetPosition(int targetPosition, double power){
        lift.setTargetPosition(targetPosition);
        setLiftPower(power);
        return true;
    }

    public boolean spin(){
        if(spinAtOrigin){
            spin.setPosition(SPIN_ROTATED);
        }else{
            spin.setPosition(SPIN_START);
        }
        spinAtOrigin = !spinAtOrigin;
        return true;
    }
    public boolean spinPosition(){
        return spinAtOrigin;
    }
    public int returnLiftPosition(){ return lift.getCurrentPosition(); }
}
