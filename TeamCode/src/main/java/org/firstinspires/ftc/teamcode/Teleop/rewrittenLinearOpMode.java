package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Glyph.IGlyph;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.twoWheelIntake;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.IUltrasonic;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.RevRangeSensor;

/**
 * Created by ishaa on 1/28/2018.
 */

@TeleOp(name = "Lefty Teleop")
public class rewrittenLinearOpMode extends LinearOpMode {
    DcMotor lf, lb, rf, rb, lift, relic_extension;
    CRServo rightWheel1, leftWheel1, rightWheel2, leftWheel2;
    Servo spin, pan, tilt, relic_claw, relic_arm, relic_tilt;
    LynxI2cColorRangeSensor glyphSensor1;
    LynxI2cColorRangeSensor glyphSensor2;
    DcMotor led_motor;
    LED leds;
    ElapsedTime intake1Time;
    ElapsedTime intake2Time;
    ElapsedTime rotateTime;
    double pitch, roll, pivot;
    int liftPosition;
    private enum glyphLiftStates{
        MANUAL, POSITION
    }
    enum intakeState{
        NOTHING,
        INTAKE_MOTOR,
        INTAKE_NO_MOTOR,
        OUTAKE
    }
    private enum glyphRotateStates{
        LIFTING, ROTATING, LOWERING, STOPPED
    }
    glyphLiftStates liftState;
    DigitalChannel glyphLimit;
    glyphRotateStates rotateState;
    boolean yPressed;
    boolean aPressed;
    boolean xPressed;

    boolean spined = false;
    boolean rotateLower = false;
    boolean clawClosed = true;

    final double LIFT_POWER = 1;
    final double LIFT_HALF_POWER_DOWN = .1;
    final double LIFT_HALF_POWER_UP = .75;
    final int LIFT_UPPER_LIMIT = 2300;
    final int GLYPH_POSITION_0 = 0;
    final int GLYPH_POSITION_1 = 200;
    final int GLYPH_POSITION_2 = 900;
    final int GLYPH_POSITION_3 = 1600;
    final int GLYPH_POSITION_4 = 2300;
    final int LIFT_POSITION_OFFSET = 100;
    final int LIFT_INTAKEN_POSITION = 250;
    final int GLYPH_ROTATE_POSITION = 800;

    final double ANALOG_PRESSED = .2;

    final double INTAKE_POWER = .74;
    final double OUTAKE_POWER = -.74;
    final double GLYPH_GRAB_DISTANCE = 5.6;
    final double GLYPH_VISIBLE_TIME = 250;

    final double SPIN_NORMAL_POSITION = .825;
    final double SPIN_SPUN_POSITION = 0;
    final double ROTATE_TIME = 750;

    final double JEWEL_TILT_POSITION = 1;
    final double JEWEL_PAN_POSITION = .55;

    boolean intaking = false;
    boolean intakePressed = false;

    final double DRIVE_LOW_SPEED = .5;

    final double RELIC_ARM_ORIGIN = .01;
    final double RELIC_ARM_GRAB_POS = .82;
    final double RELIC_ARM_EXTENSION_HALF_POWER = .5;
    final double RELIC_ARM_RETRACTION_HALF_POWER = -.5;
    final double RELIC_ARM_EXTENSION_FULL_POWER = 1;
    final double RELIC_ARM_RETRACTION_FULL_POWER = -1;

    rewritten.intakeState lowerIntakeState;
    rewritten.intakeState upperIntakeState;

    IGlyph bottomIntake, topIntake;
    IUltrasonic glyphColor1;
    IUltrasonic glyphColor2;
    ClawThreePoint relic;
    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lift = hardwareMap.dcMotor.get("glyph_lift");
        relic_extension = hardwareMap.dcMotor.get("relic_extension");

        rightWheel1 = hardwareMap.crservo.get("right_glyph1");
        leftWheel1 = hardwareMap.crservo.get("left_glyph1");
        rightWheel2 = hardwareMap.crservo.get("right_glyph2");
        leftWheel2 = hardwareMap.crservo.get("left_glyph2");

        spin = hardwareMap.servo.get("spin_grip");
        pan = hardwareMap.servo.get("jewel_pan");
        tilt = hardwareMap.servo.get("jewel_tilt");
        relic_arm = hardwareMap.servo.get("relic_arm");
        relic_claw = hardwareMap.servo.get("relic_claw");
        relic_tilt = hardwareMap.servo.get("relic_tilt");

        leftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        glyphSensor1 = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor1");
        glyphSensor2 = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor2");
        glyphColor1 = new RevRangeSensor(glyphSensor1);
        glyphColor2 = new RevRangeSensor(glyphSensor2);

        led_motor = hardwareMap.dcMotor.get("leds");
        leds = new LED(led_motor);

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relic_extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        relic_extension.setDirection(DcMotorSimple.Direction.REVERSE);

        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relic_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        glyphLimit = hardwareMap.digitalChannel.get("glyph_limit");

        liftState = glyphLiftStates.POSITION;
        lowerIntakeState = rewritten.intakeState.NOTHING;
        upperIntakeState = rewritten.intakeState.NOTHING;
        rotateState = glyphRotateStates.STOPPED;

        yPressed = false;
        aPressed = false;

        intake1Time = new ElapsedTime();
        intake2Time = new ElapsedTime();
        rotateTime = new ElapsedTime();

        bottomIntake = new twoWheelIntake(leftWheel1, rightWheel1, INTAKE_POWER, OUTAKE_POWER);
        topIntake = new twoWheelIntake(leftWheel2, rightWheel2, INTAKE_POWER, OUTAKE_POWER);

        relic = new ClawThreePoint(relic_extension, relic_arm, relic_tilt, relic_claw);
        waitForStart();
        pan.setPosition(JEWEL_PAN_POSITION);
        tilt.setPosition(JEWEL_TILT_POSITION);
        relic.pickUpRelic();
        relic.setTiltPosition(1);
        relic_arm.setPosition(0);

        while(opModeIsActive()){


            pitch = -gamepad1.left_stick_y;
            roll = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;
            if(gamepad1.right_trigger>.2){
                rf.setPower((pitch-roll-pivot)*DRIVE_LOW_SPEED);
                rb.setPower((pitch+roll-pivot)*DRIVE_LOW_SPEED);
                lf.setPower((pitch+roll+pivot)*DRIVE_LOW_SPEED);
                lb.setPower((pitch-roll+pivot)*DRIVE_LOW_SPEED);
            }else{
                rf.setPower(pitch-roll-pivot);
                rb.setPower(pitch+roll-pivot);
                lf.setPower(pitch+roll+pivot);
                lb.setPower(pitch-roll+pivot);
            }


            switch(liftState) {
                case MANUAL:
                    telemetry.addData("state", "manual");
                    liftPosition = lift.getCurrentPosition();
                    lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    if (gamepad2.left_trigger < ANALOG_PRESSED && gamepad2.right_trigger < ANALOG_PRESSED) {
                        liftState = glyphLiftStates.POSITION;
                        telemetry.addData("case", "leaving");
                    } else if (gamepad2.right_trigger >= ANALOG_PRESSED && liftPosition < LIFT_UPPER_LIMIT) {
                        if(gamepad2.right_bumper){
                            lift.setPower(LIFT_HALF_POWER_UP);
                        }else{
                            lift.setPower(LIFT_POWER);
                        }
                        telemetry.addData("case", "up");
                    } else if (gamepad2.left_trigger >= ANALOG_PRESSED && glyphLimit.getState()) {
                        if(gamepad2.right_bumper){
                            lift.setPower(-LIFT_HALF_POWER_DOWN);
                        }else{
                            lift.setPower(-LIFT_POWER);
                        }
                        telemetry.addData("case", "down");
                    } else if (!glyphLimit.getState()) {
                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        liftPosition = 0;
                        lift.setPower(0);
                        telemetry.addData("case", "switch pressed");
                    } else if (gamepad2.a) {
                        if (liftPosition < GLYPH_POSITION_1 + LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_0;
                        } else if (liftPosition < GLYPH_POSITION_2 + LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_1;
                        } else if (liftPosition < GLYPH_POSITION_3 + LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_2;
                        } else {
                            liftPosition = GLYPH_POSITION_3;
                        }
                        aPressed = true;
                        liftState = glyphLiftStates.POSITION;
                        telemetry.addData("case", "left bumper");
                    } else if (gamepad2.y) {
                        if (liftPosition > GLYPH_POSITION_3 - LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_4;
                        } else if (liftPosition > GLYPH_POSITION_2 - LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_3;
                        } else if (liftPosition > GLYPH_POSITION_1 - LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_2;
                        } else {
                            liftPosition = GLYPH_POSITION_1;
                        }
                        yPressed = true;
                        liftState = glyphLiftStates.POSITION;
                        telemetry.addData("case", "right bumper");
                    }
                    break;
                case POSITION:
                    telemetry.addData("state", "position");
                    if (liftPosition <= 0) {
                        liftPosition = 0;
                        if (glyphLimit.getState()) {
                            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            lift.setPower(-LIFT_POWER);
                            telemetry.addData("case", "lower to switch");
                        } else {
                            lift.setPower(0);
                            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            telemetry.addData("case", "switch pressed 0");
                        }
                    } else {
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setTargetPosition(liftPosition);
                        lift.setPower(LIFT_POWER);
                        telemetry.addData("case", "running to position");
                    }
                    if (gamepad2.left_trigger > ANALOG_PRESSED || gamepad2.right_trigger > ANALOG_PRESSED) {
                        liftState = glyphLiftStates.MANUAL;
                        lift.setPower(0);
                        telemetry.addData("if", "leaving");
                    } else if (gamepad2.a) {
                        if (!aPressed) {
                            if (liftPosition < GLYPH_POSITION_1 + LIFT_POSITION_OFFSET) {
                                liftPosition = GLYPH_POSITION_0;
                            } else if (liftPosition < GLYPH_POSITION_2 + LIFT_POSITION_OFFSET) {
                                liftPosition = GLYPH_POSITION_1;
                            } else if (liftPosition < GLYPH_POSITION_3 + LIFT_POSITION_OFFSET) {
                                liftPosition = GLYPH_POSITION_2;
                            } else {
                                liftPosition = GLYPH_POSITION_3;
                            }
                            aPressed = true;

                        }
                        telemetry.addData("if", "left bumper");
                    } else if (gamepad2.y) {
                        if (!yPressed) {
                            if (liftPosition > GLYPH_POSITION_3 - LIFT_POSITION_OFFSET) {
                                liftPosition = GLYPH_POSITION_4;
                            } else if (liftPosition > GLYPH_POSITION_2 - LIFT_POSITION_OFFSET) {
                                liftPosition = GLYPH_POSITION_3;
                            } else if (liftPosition > GLYPH_POSITION_1 - LIFT_POSITION_OFFSET) {
                                liftPosition = GLYPH_POSITION_2;
                            } else {
                                liftPosition = GLYPH_POSITION_1;
                            }
                            yPressed = true;
                        }
                        telemetry.addData("if", "right bumper");
                    }

                    if(!gamepad2.y){
                        yPressed = false;
                    }
                    if(!gamepad2.a){
                        aPressed = false;
                    }
                    break;
            }

            if(gamepad1.left_bumper&&!intakePressed){
                intaking = !intaking;
                intakePressed = true;
            }else if(gamepad1.left_bumper){
                intakePressed = true;
            }else{
                intakePressed = false;
            }
            switch(lowerIntakeState){
                case NOTHING:
                    if(gamepad1.right_bumper||(!spined&&gamepad1.dpad_down)||(spined&&gamepad1.dpad_up)){
                        lowerIntakeState = rewritten.intakeState.OUTAKE;
                    }else if (intaking) {
                        lowerIntakeState = rewritten.intakeState.INTAKE_MOTOR;
                        intake1Time.reset();
                    }else{
                        bottomIntake.turnOff();
                    }
                    break;

                case INTAKE_MOTOR:
                    if(glyphColor1.cmDistance() < GLYPH_GRAB_DISTANCE&&intake1Time.milliseconds()>GLYPH_VISIBLE_TIME){
                        lowerIntakeState = rewritten.intakeState.INTAKE_NO_MOTOR;
                        if(!spined&&lift.getCurrentPosition()<LIFT_INTAKEN_POSITION){
                            liftPosition = LIFT_INTAKEN_POSITION;
                        }
                    }else if(gamepad1.right_bumper||(!spined&&gamepad1.dpad_down)||(spined&&gamepad1.dpad_up)){
                        lowerIntakeState = rewritten.intakeState.OUTAKE;
                        intaking = false;
                    }else if(!intaking){
                        lowerIntakeState = rewritten.intakeState.NOTHING;
                    }else if(glyphColor1.cmDistance() > GLYPH_GRAB_DISTANCE){
                        intake1Time.reset();
                        bottomIntake.secureGlyph();
                    }else{
                        bottomIntake.secureGlyph();
                    }
                    break;
                case INTAKE_NO_MOTOR:
                    if(glyphColor1.cmDistance()>GLYPH_GRAB_DISTANCE){
                        lowerIntakeState = rewritten.intakeState.INTAKE_MOTOR;
                        intake1Time.reset();
                    }else if(gamepad1.right_bumper||(!spined&&gamepad1.dpad_down)||(spined&&gamepad1.dpad_up)){
                        lowerIntakeState = rewritten.intakeState.OUTAKE;
                        intaking = false;
                    }else if(!intaking){
                        lowerIntakeState = rewritten.intakeState.NOTHING;
                    }else{
                        bottomIntake.turnOff();
                    }
                    break;
                case OUTAKE:
                    if(!gamepad1.right_bumper&&!(!spined&&gamepad1.dpad_down)&&!(spined&&gamepad1.dpad_up)){
                        lowerIntakeState = rewritten.intakeState.NOTHING;
                    }else{
                        bottomIntake.dispenseGlyph();
                    }
                    break;
            }

            switch(upperIntakeState){
                case NOTHING:
                    if(gamepad1.right_bumper||(spined&&gamepad1.dpad_down)||(!spined&&gamepad1.dpad_up)){
                        upperIntakeState = rewritten.intakeState.OUTAKE;
                    }else if (intaking) {
                        upperIntakeState = rewritten.intakeState.INTAKE_MOTOR;
                        intake2Time.reset();
                    }else{
                        topIntake.turnOff();
                    }
                    break;

                case INTAKE_MOTOR:
                    if(glyphColor2.cmDistance() < GLYPH_GRAB_DISTANCE&&intake2Time.milliseconds()>GLYPH_VISIBLE_TIME){
                        upperIntakeState= rewritten.intakeState.INTAKE_NO_MOTOR;
                        if(spined&&lift.getCurrentPosition()<LIFT_INTAKEN_POSITION){
                            liftPosition = LIFT_INTAKEN_POSITION;
                        }
                    }else if(gamepad1.right_bumper||(spined&&gamepad1.dpad_down)||(!spined&&gamepad1.dpad_up)){
                        upperIntakeState = rewritten.intakeState.OUTAKE;
                        intaking = false;
                    }else if(!intaking){
                        upperIntakeState = rewritten.intakeState.NOTHING;
                    }else if(glyphColor1.cmDistance() > GLYPH_GRAB_DISTANCE){
                        intake2Time.reset();
                        topIntake.secureGlyph();
                    }else{
                        topIntake.secureGlyph();
                    }
                    break;
                case INTAKE_NO_MOTOR:
                    if(glyphColor2.cmDistance()>GLYPH_GRAB_DISTANCE){
                        upperIntakeState = rewritten.intakeState.INTAKE_MOTOR;
                        intake2Time.reset();
                    }else if(gamepad1.right_bumper||(spined&&gamepad1.dpad_down)||(!spined&&gamepad1.dpad_up)){
                        upperIntakeState = rewritten.intakeState.OUTAKE;
                        intaking = false;
                    }else if(!intaking){
                        upperIntakeState = rewritten.intakeState.NOTHING;
                    }else{
                        topIntake.turnOff();
                    }
                    break;
                case OUTAKE:
                    if(!gamepad1.right_bumper&&!(spined&&gamepad1.dpad_down)&&!(!spined&&gamepad1.dpad_up)){
                        upperIntakeState = rewritten.intakeState.NOTHING;
                    }else{
                        topIntake.dispenseGlyph();
                    }
                    break;
            }
            switch(rotateState){
                case STOPPED:
                    if(spined){
                        spin.setPosition(SPIN_SPUN_POSITION);
                    }else{
                        spin.setPosition(SPIN_NORMAL_POSITION);
                    }
                    if(gamepad2.left_bumper&&liftState!= glyphLiftStates.MANUAL){
                        if(liftPosition < GLYPH_ROTATE_POSITION){
                            rotateState = glyphRotateStates.LIFTING;
                            liftPosition = GLYPH_ROTATE_POSITION+LIFT_POSITION_OFFSET;
                            rotateLower = true;
                        }else{
                            rotateState = glyphRotateStates.ROTATING;
                            spined = !spined;
                            rotateTime.reset();
                            rotateLower = false;
                        }
                    }
                    break;
                case LIFTING:
                    if(liftState == glyphLiftStates.MANUAL){
                        rotateState = glyphRotateStates.STOPPED;
                    }
                    if(lift.getCurrentPosition()>=GLYPH_ROTATE_POSITION){
                        rotateState = glyphRotateStates.ROTATING;
                        spined = !spined;
                        rotateTime.reset();
                    }
                    break;
                case ROTATING:
                    if(rotateTime.milliseconds()>ROTATE_TIME){
                        if(rotateLower){
                            rotateState = glyphRotateStates.LOWERING;
                            liftPosition = 0;
                        }else{
                            rotateState = glyphRotateStates.STOPPED;
                        }
                    }else if(liftState == glyphLiftStates.MANUAL){
                        rotateState = glyphRotateStates.STOPPED;
                    }else if(spined){
                        spin.setPosition(SPIN_SPUN_POSITION);
                    }else{
                        spin.setPosition(SPIN_NORMAL_POSITION);
                    }
                    break;
                case LOWERING:
                    if(lift.getCurrentPosition()<LIFT_POSITION_OFFSET||liftState == glyphLiftStates.MANUAL){
                        rotateState = glyphRotateStates.STOPPED;
                    }
            }
            if(spined){
                if(upperIntakeState== rewritten.intakeState.INTAKE_NO_MOTOR){
                    leds.setLEDPower(.5);
                }else{
                    leds.setLEDPower(0);
                }
            }else{
                if(lowerIntakeState== rewritten.intakeState.INTAKE_NO_MOTOR){
                    leds.setLEDPower(.5);
                }else{
                    leds.setLEDPower(0);
                }
            }

            //Relic Extension Motor Controls with Encoder Limits
            if (gamepad2.right_bumper) {
                relic.extend(RELIC_ARM_EXTENSION_FULL_POWER, gamepad2.dpad_up && relic_extension.getCurrentPosition() < 2200);
            } else{
                relic.extend(RELIC_ARM_EXTENSION_HALF_POWER, gamepad2.dpad_up && relic_extension.getCurrentPosition() < 2200);
            }

            if(gamepad2.right_bumper){
                relic.retract(RELIC_ARM_RETRACTION_FULL_POWER, gamepad2.dpad_down && relic_extension.getCurrentPosition() > 200);
            }else{
                relic.retract(RELIC_ARM_RETRACTION_HALF_POWER, gamepad2.dpad_down && relic_extension.getCurrentPosition() > 200);
            }

            if(!gamepad2.dpad_up && !gamepad2.dpad_down){
                relic.extensionPowerZero();
            }

            telemetry.addData("relic extension position", relic_extension.getCurrentPosition());

            if(gamepad2.x&&!xPressed){
                if(clawClosed){
                    relic.releaseRelic();
                }else{
                    relic.pickUpRelic();
                }
                clawClosed=!clawClosed;
                xPressed = true;
            }else if(!gamepad2.x){
                xPressed = false;
            }

            if(gamepad2.dpad_left){
                relic.setArmPosition(RELIC_ARM_GRAB_POS);
            }else if(gamepad1.a){
                relic.setArmPosition(RELIC_ARM_ORIGIN);
                relic.pickUpRelic();
                relic.setTiltPosition(1);
            }
            if(gamepad2.dpad_right){
                relic.setTiltPosition(0.6);
                relic.setArmPosition(0.6);
            }
            //Relic Arm Servo Controls
            if (relic.returnArmPos()< .4) {
                relic.adjustArm((-gamepad2.right_stick_y > 0.1 && relic.returnArmPos() <= 1), 0.05);
                relic.adjustArm((-gamepad2.right_stick_y < -0.1 && relic.returnArmPos() >= 0.04), -0.05);

            } else {
                relic.adjustArm(-gamepad2.right_stick_y > 0.1 && relic.returnArmPos() <= 1, .005);
                relic.adjustArm(-gamepad2.right_stick_y < -0.1 && relic.returnArmPos() >= 0.04, -.005);
            }

            telemetry.addData("Lift Position", lift.getCurrentPosition());
            telemetry.update();
        }


    }
}
