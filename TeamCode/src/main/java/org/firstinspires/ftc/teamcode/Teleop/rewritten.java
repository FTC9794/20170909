package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by ishaa on 1/19/2018.
 */
@TeleOp(name = "rewritten teleop", group = "teleop")
public class rewritten extends OpMode {
    DcMotor lf, lb, rf, rb;
    double pitch, roll, pivot;
    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        pitch = -gamepad1.left_stick_y;
        roll = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;
        rf.setPower(pitch-roll-pivot);
        rb.setPower(pitch+roll-pivot);
        lf.setPower(pitch+roll+pivot);
        lb.setPower(pitch-roll+pivot);
        telemetry.addData("pitch", pitch);
        telemetry.addData("roll", roll);
        telemetry.addData("pivot", pivot);
    }
}
