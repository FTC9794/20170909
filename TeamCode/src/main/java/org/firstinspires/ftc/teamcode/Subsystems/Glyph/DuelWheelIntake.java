package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Sarthak on 11/11/2017.
 */

public class DuelWheelIntake implements IGlyph {
    private CRServo rightWheel1, leftWheel1;
    private DcMotor lift;
    Telemetry telemetry;
    private final double INTAKE_SPPED = 1;
    private final double OUTTAKE_SPEED = -1;

    public DuelWheelIntake(CRServo rightWheel1, CRServo leftWheel1, DcMotor lift, Telemetry telemetry){
        this.rightWheel1 = rightWheel1;
        this.rightWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftWheel1 = leftWheel1;
        this.telemetry = telemetry;
        this.lift = lift;
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

    @Override
    public void changeHeight(double power, boolean condition) {
        if(condition){
            lift.setPower(power);
        }else{
            lift.setPower(0);
        }
    }
}
