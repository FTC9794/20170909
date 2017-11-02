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
    boolean spinPressed = false;
    boolean gripPressed1 = false;
    boolean gripPressed2 = false;
    enum liftState{
        MANUAL,
        POSITION
    }
    liftState glyphLiftState;
    Servo pan, tilt;

    double liftIncriment = 0;
    DigitalChannel glyphLimit;

    final double gripOpen1 = .5;
    final double gripOpen2 = .5;
    final double gripClose1 = 0;
    final double gripClose2 = 0;

    final double spinStart = 0;
    final double spinRotated = .95;

    final int liftPosition1 = 800;
    final int liftPosition2 = 1000;
    final int liftPosition3 = 1200;
    final int liftPosition4 = 1600;

    FourArmRotatingGlyph glyph;

    boolean bothOpened1 = true;
    boolean bothOpened2 = true;
    boolean spinAtOrigin = true;

    DcMotor lift;

    @Override
    public void runOpMode() throws InterruptedException {
        right1 = hardwareMap.servo.get("right_glyph1");
        right2 = hardwareMap.servo.get("right_glyph2");
        left1 = hardwareMap.servo.get("left_glyph1");
        left2 = hardwareMap.servo.get("left_glyph2");
        spin = hardwareMap.servo.get("spin_grip");

        glyphLiftState = liftState.MANUAL;
        pan = hardwareMap.servo.get("jewel_pan");
        tilt = hardwareMap.servo.get("jewel_tilt");

        lift = hardwareMap.dcMotor.get("glyph_lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right2.setDirection(Servo.Direction.REVERSE);
        left1.setDirection(Servo.Direction.REVERSE);

        right1.setPosition(gripOpen1);
        right2.setPosition(gripOpen2);
        left1.setPosition(gripOpen1);
        left2.setPosition(gripOpen2);

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
                if(!spinPressed){
                    if(spinAtOrigin) {

                        spin.setPosition(spinRotated);
                        spinAtOrigin = false;
                    }else{
                        spin.setPosition(spinStart);
                        spinAtOrigin = true;
                    }
                }
                spinPressed = true;
            }else{
                spinPressed = false;
            }
            if(gamepad1.x){
                if(!gripPressed2){
                    if(bothOpened2){
                        right2.setPosition(gripClose2);
                        left2.setPosition(gripClose2);
                        bothOpened2 = false;
                    }else{
                        right2.setPosition(gripOpen2);
                        left2.setPosition(gripOpen2);
                        bothOpened2 = true;
                    }
                }
                gripPressed2 = true;
            }else{
                gripPressed2 = false;
            }
            if(gamepad1.y){
                if(!gripPressed1){
                    if(bothOpened1){
                        right1.setPosition(gripClose1);
                        left1.setPosition(gripClose1);

                        bothOpened1 = false;
                    }else{
                        right1.setPosition(gripOpen1);
                        left1.setPosition(gripOpen1);
                        bothOpened1 = true;
                    }
                }
                gripPressed1 = true;
            }else{
                gripPressed1 = false;
            }

            switch(glyphLiftState){
                case MANUAL:
                    if(gamepad1.left_trigger>.5&&glyphLimit.getState()){
                        lift.setPower(-1);
                    }else if(gamepad1.right_trigger>.5&&lift.getCurrentPosition()<liftPosition4){
                        lift.setPower(.75);
                    }else if(gamepad1.left_bumper){
                        if(lift.getCurrentPosition()<liftPosition1){
                            liftIncriment = 0;
                        }else if(lift.getCurrentPosition()<liftPosition2){
                            liftIncriment = 1;
                        }else if(lift.getCurrentPosition()<liftPosition3){
                            liftIncriment = 2;
                        }else {
                            liftIncriment = 3;
                        }
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        glyphLiftState = liftState.POSITION;
                    }else if(gamepad1.right_bumper){
                        if(lift.getCurrentPosition()<liftPosition1){
                            liftIncriment = 1;
                        }else if(lift.getCurrentPosition()<liftPosition2){
                            liftIncriment = 2;
                        }else if(lift.getCurrentPosition()<liftPosition3){
                            liftIncriment = 3;
                        }else {
                            liftIncriment = 4;
                        }
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        glyphLiftState = liftState.POSITION;
                    }

                    break;
                case POSITION:
                    if(gamepad1.left_bumper&&liftIncriment>0){
                        liftIncriment--;
                    }
                    if(gamepad1.right_bumper&&liftIncriment<4){
                        liftIncriment++;
                    }
                    if(liftIncriment==1){
                        lift.setTargetPosition(liftPosition1);
                    }else if(liftIncriment==2){
                        lift.setTargetPosition(liftPosition2);
                    }else if(liftIncriment==3){
                        lift.setTargetPosition(liftPosition3);
                    }else if(liftIncriment==4){
                        lift.setTargetPosition(liftPosition4);
                    }else if(liftIncriment==0){
                        lift.setTargetPosition(0);
                    }
                    if(!lift.isBusy()){
                        glyphLiftState = liftState.MANUAL;
                    }else if(gamepad1.left_trigger>.5||gamepad1.right_trigger>.5){
                        glyphLiftState = liftState.MANUAL;
                    }
                    lift.setPower(.5);
                    break;
            }

            telemetry.addData("Lift Value", lift.getCurrentPosition());
            telemetry.update();
        }

    }
}
