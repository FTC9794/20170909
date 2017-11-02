package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Glyph.FourArmRotatingGlyph;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.IGlyph;

/**
 * Created by Sarthak on 11/1/2017.
 */
@TeleOp(name = "The Wizard Teleop", group = "Teleop")
public class TheWizardTeleop extends LinearOpMode {
    Servo right1;
    Servo right2;
    Servo left1;
    Servo left2;
    Servo spin;

    Servo pan, tilt;

    DigitalChannel glyphLimit;

    final double gripOpen1 = .5;
    final double gripOpen2 = .5;
    final double gripClose1 = 0;
    final double gripClose2 = 0;

    final double spinStart = 0;
    final double spinRotated = .95;

    FourArmRotatingGlyph glyph;

    boolean bothOpened = false;
    boolean spinAtOrigin = true;

    DcMotor lift;

    @Override
    public void runOpMode() throws InterruptedException {
        right1 = hardwareMap.servo.get("right_glyph1");
        right2 = hardwareMap.servo.get("right_glyph2");
        left1 = hardwareMap.servo.get("left_glyph1");
        left2 = hardwareMap.servo.get("left_glyph2");
        spin = hardwareMap.servo.get("spin_grip");


        pan = hardwareMap.servo.get("jewel_pan");
        tilt = hardwareMap.servo.get("jewel_tilt");

        lift = hardwareMap.dcMotor.get("glyph_lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right2.setDirection(Servo.Direction.REVERSE);
        left1.setDirection(Servo.Direction.REVERSE);

        right1.setPosition(gripClose1);
        right2.setPosition(gripClose2);
        left1.setPosition(gripClose1);
        left2.setPosition(gripClose2);

        spin.setPosition(spinStart);
        glyphLimit = hardwareMap.digitalChannel.get("glyph_limit");
        pan.setPosition(.5);
        tilt.setPosition(1);

        glyph = new FourArmRotatingGlyph(right1, right2, left1, left2, spin, lift);

        waitForStart();
        while(opModeIsActive()){
            if(!glyphLimit.getState()){
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addData("Encoder", "Reset");
            }
            if(gamepad1.a){
                if(spinAtOrigin) {
                    while(gamepad1.a);
                    spin.setPosition(spinRotated);
                    spinAtOrigin = false;
                }else{
                    while(gamepad1.a);
                    spin.setPosition(spinStart);
                    spinAtOrigin = true;

                }
            }
            if(gamepad1.y){
                while (gamepad1.y);
                if(bothOpened){
                    right1.setPosition(gripClose1);
                    left1.setPosition(gripClose1);
                    right2.setPosition(gripClose2);
                    left2.setPosition(gripClose2);
                    bothOpened = false;
                }else{
                    right1.setPosition(gripOpen1);
                    left1.setPosition(gripOpen1);
                    right2.setPosition(gripOpen2);
                    left2.setPosition(gripOpen2);
                    bothOpened = true;
                }
            }
            if(gamepad1.left_trigger>.5&&glyphLimit.getState()){
                lift.setPower(-1);
            }else if(gamepad1.right_trigger>.5){
                lift.setPower(.75);
            }else{
                lift.setPower(0);
            }

            telemetry.addData("Lift Value", lift.getCurrentPosition());
            telemetry.update();
        }

    }
}
