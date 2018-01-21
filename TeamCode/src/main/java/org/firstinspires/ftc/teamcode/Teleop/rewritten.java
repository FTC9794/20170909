package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Glyph.IGlyph;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.twoWheelIntake;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.IUltrasonic;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.RevRangeSensor;

/**
 * Created by ishaa on 1/19/2018.
 */
@TeleOp(name = "rewritten teleop", group = "teleop")
public class rewritten extends OpMode {
    DcMotor lf, lb, rf, rb, lift;
    CRServo rightWheel1, leftWheel1, rightWheel2, leftWheel2;
    LynxI2cColorRangeSensor glyphSensor1;
    LynxI2cColorRangeSensor glyphSensor2;
    DcMotor led_motor;
    LED leds;
    ElapsedTime intake1Time;
    ElapsedTime intake2Time;
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
    glyphLiftStates liftState;
    DigitalChannel glyphLimit;

    boolean rightBumpPressed;
    boolean leftBumpPressed;

    final double LIFT_POWER = 1;
    final int LIFT_UPPER_LIMIT = 2300;
    final int GLYPH_POSITION_0 = 0;
    final int GLYPH_POSITION_1 = 160;
    final int GLYPH_POSITION_2 = 840;
    final int GLYPH_POSITION_3 = 1460;
    final int GLYPH_POSITION_4 = 2230;
    final int LIFT_POSITION_OFFSET = 100;

    final double ANALOG_PRESSED = .2;

    final double INTAKE_POWER = .74;
    final double OUTAKE_POWER = -.74;
    final double GLYPH_GRAB_DISTANCE = 5.6;
    final double GLYPH_VISIBLE_TIME = 250;

    boolean intaking = false;
    boolean intakePressed = false;

    intakeState lowerIntakeState;
    intakeState upperIntakeState;

    IGlyph bottomIntake, topIntake;
    IUltrasonic glyphColor1;
    IUltrasonic glyphColor2;
    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lift = hardwareMap.dcMotor.get("glyph_lift");

        rightWheel1 = hardwareMap.crservo.get("right_glyph1");
        leftWheel1 = hardwareMap.crservo.get("left_glyph1");
        rightWheel2 = hardwareMap.crservo.get("right_glyph2");
        leftWheel2 = hardwareMap.crservo.get("left_glyph2");

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
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        glyphLimit = hardwareMap.digitalChannel.get("glyph_limit");

        liftState = glyphLiftStates.POSITION;
        lowerIntakeState = intakeState.NOTHING;
        upperIntakeState = intakeState.NOTHING;

        rightBumpPressed = false;
        leftBumpPressed = false;

        intake1Time = new ElapsedTime();
        intake2Time = new ElapsedTime();

        bottomIntake = new twoWheelIntake(leftWheel1, rightWheel1, INTAKE_POWER, OUTAKE_POWER);
        topIntake = new twoWheelIntake(leftWheel2, rightWheel2, INTAKE_POWER, OUTAKE_POWER);
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

        switch(liftState) {
            case MANUAL:
                telemetry.addData("state", "manual");
                liftPosition = lift.getCurrentPosition();
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (gamepad2.left_trigger < ANALOG_PRESSED && gamepad2.right_trigger < ANALOG_PRESSED) {
                    liftState = glyphLiftStates.POSITION;
                    telemetry.addData("case", "leaving");
                } else if (gamepad2.right_trigger >= ANALOG_PRESSED && liftPosition < LIFT_UPPER_LIMIT) {
                    lift.setPower(LIFT_POWER);
                    telemetry.addData("case", "up");
                } else if (gamepad2.left_trigger >= ANALOG_PRESSED && glyphLimit.getState()) {
                    lift.setPower(-LIFT_POWER);
                    telemetry.addData("case", "down");
                } else if (!glyphLimit.getState()) {
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftPosition = 0;
                    lift.setPower(0);
                    telemetry.addData("case", "switch pressed");
                } else if (gamepad2.left_bumper) {
                    if (liftPosition < GLYPH_POSITION_1 + LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_0;
                    } else if (liftPosition < GLYPH_POSITION_2 + LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_1;
                    } else if (liftPosition < GLYPH_POSITION_3 + LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_2;
                    } else {
                        liftPosition = GLYPH_POSITION_3;
                    }
                    leftBumpPressed = true;
                    liftState = glyphLiftStates.POSITION;
                    telemetry.addData("case", "left bumper");
                } else if (gamepad2.right_bumper) {
                    if (liftPosition > GLYPH_POSITION_3 - LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_4;
                    } else if (liftPosition > GLYPH_POSITION_2 - LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_3;
                    } else if (liftPosition > GLYPH_POSITION_1 - LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_2;
                    } else {
                        liftPosition = GLYPH_POSITION_1;
                    }
                    rightBumpPressed = true;
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
                } else if (gamepad2.left_bumper) {
                    if (!leftBumpPressed) {
                        if (liftPosition < GLYPH_POSITION_1 + LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_0;
                        } else if (liftPosition < GLYPH_POSITION_2 + LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_1;
                        } else if (liftPosition < GLYPH_POSITION_3 + LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_2;
                        } else {
                            liftPosition = GLYPH_POSITION_3;
                        }
                        leftBumpPressed = true;

                    }
                    telemetry.addData("if", "left bumper");
                } else if (gamepad2.right_bumper) {
                    if (!rightBumpPressed) {
                        if (liftPosition > GLYPH_POSITION_3 - LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_4;
                        } else if (liftPosition > GLYPH_POSITION_2 - LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_3;
                        } else if (liftPosition > GLYPH_POSITION_1 - LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_2;
                        } else {
                            liftPosition = GLYPH_POSITION_1;
                        }
                        rightBumpPressed = true;
                    }
                    telemetry.addData("if", "right bumper");
                }

                if(!gamepad2.right_bumper){
                    rightBumpPressed = false;
                }
                if(!gamepad2.left_bumper){
                    leftBumpPressed = false;
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
                if(gamepad1.right_bumper){
                    lowerIntakeState = intakeState.OUTAKE;
                }else if (intaking) {
                    lowerIntakeState = intakeState.INTAKE_MOTOR;
                    intake1Time.reset();
                }else{
                    bottomIntake.turnOff();
                }
                break;

            case INTAKE_MOTOR:
                if(glyphColor1.cmDistance() < GLYPH_GRAB_DISTANCE&&intake1Time.milliseconds()>GLYPH_VISIBLE_TIME){
                    lowerIntakeState = intakeState.INTAKE_NO_MOTOR;
                }else if(gamepad1.right_bumper){
                    lowerIntakeState = intakeState.OUTAKE;
                    intaking = false;
                }else if(!intaking){
                    lowerIntakeState = intakeState.NOTHING;
                }else if(glyphColor1.cmDistance() > GLYPH_GRAB_DISTANCE){
                    intake1Time.reset();
                    bottomIntake.secureGlyph();
                }else{
                    bottomIntake.secureGlyph();
                }
                break;
            case INTAKE_NO_MOTOR:
                if(glyphColor1.cmDistance()>GLYPH_GRAB_DISTANCE){
                    lowerIntakeState = intakeState.INTAKE_MOTOR;
                    intake1Time.reset();
                }else if(gamepad1.right_bumper){
                    lowerIntakeState = intakeState.OUTAKE;
                    intaking = false;
                }else if(!intaking){
                    lowerIntakeState = intakeState.NOTHING;
                }else{
                    bottomIntake.turnOff();
                }
                break;
            case OUTAKE:
                if(!gamepad1.right_bumper){
                    lowerIntakeState = intakeState.NOTHING;
                }else{
                    bottomIntake.dispenseGlyph();
                }
                break;
        }

        switch(upperIntakeState){
            case NOTHING:
                if(gamepad1.right_bumper){
                    upperIntakeState = intakeState.OUTAKE;
                }else if (intaking) {
                    upperIntakeState = intakeState.INTAKE_MOTOR;
                    intake2Time.reset();
                }else{
                    topIntake.turnOff();
                }
                break;

            case INTAKE_MOTOR:
                if(glyphColor2.cmDistance() < GLYPH_GRAB_DISTANCE&&intake2Time.milliseconds()>GLYPH_VISIBLE_TIME){
                    upperIntakeState= intakeState.INTAKE_NO_MOTOR;
                }else if(gamepad1.right_bumper){
                    upperIntakeState = intakeState.OUTAKE;
                    intaking = false;
                }else if(!intaking){
                    upperIntakeState = intakeState.NOTHING;
                }else if(glyphColor1.cmDistance() > GLYPH_GRAB_DISTANCE){
                    intake2Time.reset();
                    topIntake.secureGlyph();
                }else{
                    topIntake.secureGlyph();
                }
                break;
            case INTAKE_NO_MOTOR:
                if(glyphColor2.cmDistance()>GLYPH_GRAB_DISTANCE){
                    upperIntakeState = intakeState.INTAKE_MOTOR;
                    intake2Time.reset();
                }else if(gamepad1.right_bumper){
                    upperIntakeState = intakeState.OUTAKE;
                    intaking = false;
                }else if(!intaking){
                    upperIntakeState = intakeState.NOTHING;
                }else{
                    topIntake.turnOff();
                }
                break;
            case OUTAKE:
                if(!gamepad1.right_bumper){
                    upperIntakeState = intakeState.NOTHING;
                }else{
                    topIntake.dispenseGlyph();
                }
                break;
        }


    }
}
