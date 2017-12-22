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

    private final double INTAKE_SPPED = 1;
    private final double OUTTAKE_SPEED = -1;
    final double SPIN_START = 1;
    final double SPIN_ROTATED = 0;

    boolean spinAtOrigin = true;

    public DualWheelIntake(CRServo rightWheel1, CRServo rightWheel2, CRServo leftWheel1, CRServo leftWheel2, Servo spin, DcMotor lift, DigitalChannel glyphLimit, Telemetry telemetry){
        this.rightWheel1 = rightWheel1;
        this.rightWheel2 = rightWheel2;
        this.leftWheel1 = leftWheel1;
        this.leftWheel2 = leftWheel2;

        leftWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        this.spin = spin;
        this.telemetry = telemetry;
        this.lift = lift;
        this.glyphLimit = glyphLimit;

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        spin.setPosition(SPIN_START);
    }

    @Override
    public void secureGlyph() {
        rightWheel1.setPower(INTAKE_SPPED);
        leftWheel1.setPower(INTAKE_SPPED);
    }

    @Override
    public void dispenseGlyph() {
        rightWheel1.setPower(OUTTAKE_SPEED);
        leftWheel1.setPower(OUTTAKE_SPEED);
    }

    public void setIntakePowerZero(){
        rightWheel1.setPower(0);
        rightWheel2.setPower(0);
        leftWheel1.setPower(0);
        leftWheel2.setPower(0);
    }

    @Override
    public void changeHeight(double power, boolean condition) {
        if(condition){
            lift.setPower(power);
        }
    }

    public void setLiftPowerZero(boolean condition){
        if(condition){
            lift.setPower(0);
        }
    }

    public void checkGlyphLiftLimit(){
        if (!glyphLimit.getState()) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

    public void spin(){
        if(spinAtOrigin){
            spin.setPosition(SPIN_ROTATED);
        }else{
            spin.setPosition(SPIN_START);
        }
        spinAtOrigin = !spinAtOrigin;
    }

    public int returnLiftPosition(){ return lift.getCurrentPosition(); }
}
