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

import org.firstinspires.ftc.teamcode.GamepadPlus;
import org.firstinspires.ftc.teamcode.Handiness;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.OmniDirectionalDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Jewel.TwoPointJewelArm;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.IUltrasonic;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.RevRangeSensor;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Sarthak on 11/1/2017.
 */
@TeleOp(name = "The Wizard Teleop Lefty", group = "Teleop")
public class TheWizardTeleopLefty extends LinearOpMode {
    ElapsedTime rotateTime;

    GamepadPlus gamepadPlus1;
    GamepadPlus gamepadPlus2;

    Servo spin;
    Servo pan, tilt;
    Servo relic_claw, relic_arm, relic_tilt;
    CRServo rightWheel1, leftWheel1, rightWheel2, leftWheel2;
    DigitalChannel glyphLimit;
    DcMotor lift, relic_extension;
    DcMotor rf, rb, lf, lb;
    IUltrasonic glyphColor1;
    IUltrasonic glyphColor2;
    LynxI2cColorRangeSensor glyphSensor1;
    LynxI2cColorRangeSensor glyphSensor2;
    DcMotor led_motor;
    LED leds;

    //double thrust, sideways, pivot, rfPower, rbPower, lfPower, lbPower;

    ClawThreePoint relic;
    DualWheelIntake intake;
    OmniDirectionalDrive drive;
    TwoPointJewelArm jewel;

    List<DcMotor> driveMotors;

    boolean hasSpinned = false;
    boolean spinPressed = false;
    boolean incrementPressed = false;
    boolean decrementPressed = false;
    boolean lowerLift = false;
    boolean intakeDirection = false;
    boolean intakePressed = false;

    boolean limitSwitchPressed = false;
    int desiredEncoderPosition = 0;

    enum liftState {
        MANUAL,
        POSITION
    }

    liftState glyphLiftState;

    enum rotateState {
        MANUAL,
        LIFTING,
        ROTATING,
        LOWERING
    }

    rotateState glyphRotateState;

    double liftIncriment = 0;


    final double SPIN_START = 1;
    final double SPIN_ROTATED = 0;

    final int LIFT_POSITION1 = 130;
    final int LIFT_POSITION2 = 357;
    final int LIFT_POSITION3 = 750;
    final int LIFT_POSITION4 = 1100;
    final int ROTATE_POSITION = 600;

    final int[] LIFT_POSITIONS = {160, 840, 1460, 2230};

    final double ANALOG_PRESSED = .5;

    final double LIFT_GRACE_AREA = 100;

    final double LIFT_POWER_UP = 1;
    final double LIFT_POWER_DOWN = -1;

    final double JEWEL_PAN_START = .55;
    final double JEWEL_TILT_START = 1;

    final double ROTATION_TIME = 250;



    final double RELIC_ARM_ORIGIN = 0;
    final double RELIC_ARM_GRAB_POS = .86;

    final double RELIC_ARM_EXTENSION_POWER = 1;
    final double RELIC_ARM_RETRACTION_POWER = -1;

    Handiness hand = Handiness.LEFT;
    boolean selectedHand = false;
    boolean intakePowerOff = true;

    @Override
    public void runOpMode() throws InterruptedException {

        spin = hardwareMap.servo.get("spin_grip");

        pan = hardwareMap.servo.get("jewel_pan");
        tilt = hardwareMap.servo.get("jewel_tilt");

        lift = hardwareMap.dcMotor.get("glyph_lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        glyphLimit = hardwareMap.digitalChannel.get("glyph_limit");

        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors = new ArrayList<>();
        driveMotors.add(rf);
        driveMotors.add(rb);
        driveMotors.add(lf);
        driveMotors.add(lb);

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        relic_extension = hardwareMap.dcMotor.get("relic_extension");
        relic_extension.setDirection(DcMotorSimple.Direction.REVERSE);
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relic_extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relic_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        relic_arm = hardwareMap.servo.get("relic_arm");
        relic_claw = hardwareMap.servo.get("relic_claw");
        relic_tilt = hardwareMap.servo.get("relic_tilt");

        rightWheel1 = hardwareMap.crservo.get("right_glyph1");
        leftWheel1 = hardwareMap.crservo.get("left_glyph1");
        rightWheel2 = hardwareMap.crservo.get("right_glyph2");
        leftWheel2 = hardwareMap.crservo.get("left_glyph2");
        glyphLiftState = liftState.MANUAL;
        glyphRotateState = rotateState.MANUAL;

        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relic_extension.setDirection(DcMotorSimple.Direction.REVERSE);

        rotateTime = new ElapsedTime();
        ElapsedTime ledTime = new ElapsedTime();
        gamepadPlus1 = new GamepadPlus(gamepad1);
        gamepadPlus2 = new GamepadPlus(gamepad2);

        glyphSensor1 = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor1");
        glyphSensor2 = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor2");
        glyphColor1 = new RevRangeSensor(glyphSensor1);
        glyphColor2 = new RevRangeSensor(glyphSensor2);

        led_motor = hardwareMap.dcMotor.get("leds");
        leds = new LED(led_motor);

        telemetry.addData("Initialized", "Done");
        telemetry.addData("Hand Selected", hand);
        telemetry.update();

        waitForStart();
        relic = new ClawThreePoint(relic_extension, relic_arm, relic_tilt, relic_claw, telemetry);
        intake = new DualWheelIntake(rightWheel1, rightWheel2, leftWheel1, leftWheel2, spin, lift, glyphLimit, telemetry);
        drive = new OmniDirectionalDrive(driveMotors, telemetry);
        jewel = new TwoPointJewelArm(pan, tilt, null, telemetry);

        relic.pickUpRelic();
        relic.setTiltPosition(1);

        //pan.setPosition(JEWEL_PAN_START);
        //tilt.setPosition(JEWEL_TILT_START);
        jewel.setPanTiltPos(JEWEL_PAN_START, JEWEL_TILT_START);
        relic.setArmPosition(RELIC_ARM_ORIGIN);
        desiredEncoderPosition = intake.returnLiftPosition();
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ElapsedTime liftTime = new ElapsedTime();
        ledTime.reset();

        while (opModeIsActive()) {

            if(glyphColor1.cmDistance() < 6){
                telemetry.addLine("YOU HAVE A BOTTOM GLYPH");
                leds.turnOn();
                intakePowerOff = true;
                ledTime.reset();
            }else if(glyphColor2.cmDistance() < 6){
                telemetry.addLine("YOU HAVE A TOP GLYPH");
            }else{
                leds.turnOff();
            }

            //Glyph rotation state machine
            switch(glyphRotateState){
                case MANUAL:
                    if(gamepadPlus2.leftBumper()){
                        if(!spinPressed){
                            /*if(lift.getCurrentPosition()>ROTATE_POSITION) {
                                glyphRotateState = rotateState.ROTATING;
                                rotateTime.reset();
                                lowerLift = false;
                                hasSpinned = false;
                            }else{
                                glyphLiftState = liftState.POSITION;
                                liftIncriment = 1.5;
                                liftTime.reset();
                                glyphRotateState = rotateState.LIFTING;
                                lowerLift = true;
                                hasSpinned = false;
                                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            }*/
                            if(lift.getCurrentPosition()>ROTATE_POSITION) {
                                intake.spin();
                            }
                        }
                        spinPressed = true;
                    }else{
                        spinPressed = false;
                    }
                    break;

                case ROTATING:
                    if(!hasSpinned){
                        intake.spin();
                        hasSpinned = true;
                    }else{
                        hasSpinned = true;
                    }

                    if(rotateTime.milliseconds()>ROTATION_TIME){
                        if(lowerLift){
                            glyphRotateState = rotateState.LOWERING;
                            liftTime.reset();
                            glyphLiftState = liftState.POSITION;
                            liftIncriment = 0;
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            hasSpinned = false;
                        }else{
                            glyphRotateState = rotateState.MANUAL;
                            hasSpinned = false;
                        }
                    }
                    break;

                case LIFTING:
                    desiredEncoderPosition = LIFT_POSITIONS[1];
                    if(gamepadPlus2.leftTrigger()>ANALOG_PRESSED||gamepadPlus2.rightTrigger()>ANALOG_PRESSED) {
                        glyphRotateState = rotateState.MANUAL;
                        desiredEncoderPosition = intake.returnLiftPosition();
                    }else if(!lift.isBusy() || liftTime.milliseconds() > 1750){
                        glyphRotateState = rotateState.ROTATING;
                        rotateTime.reset();
                    }
                    break;

                case LOWERING:
                    desiredEncoderPosition = 0;
                    if(gamepadPlus2.leftTrigger()>ANALOG_PRESSED||gamepadPlus2.rightTrigger()>ANALOG_PRESSED) {
                        glyphRotateState = rotateState.MANUAL;
                        desiredEncoderPosition = intake.returnLiftPosition();
                    }else if(!lift.isBusy() || liftTime.milliseconds() > 1750 || !glyphLimit.getState()){
                        glyphRotateState = rotateState.MANUAL;
                    }
                    break;
            }

            //Glyph Lift State Machine
            /*switch (glyphLiftState) {
                case MANUAL:
                    intake.checkGlyphLiftLimit();

                    intake.changeHeight(LIFT_POWER_DOWN, gamepadPlus2.leftTrigger() > ANALOG_PRESSED && glyphLimit.getState());
                    intake.changeHeight(LIFT_POWER_DOWN * 0.75, (gamepadPlus2.leftTrigger() > ANALOG_PRESSED && glyphLimit.getState() && gamepadPlus2.rightBumper()));
                    intake.changeHeight(LIFT_POWER_UP, gamepadPlus2.rightTrigger() > ANALOG_PRESSED && intake.returnLiftPosition() < 1450);
                    intake.changeHeight(LIFT_POWER_UP * 0.75, gamepadPlus2.rightTrigger() > ANALOG_PRESSED && intake.returnLiftPosition() < 1450 && gamepadPlus2.rightBumper());
                    intake.setLiftPowerZero(!(gamepadPlus2.rightTrigger() > ANALOG_PRESSED) && !(gamepadPlus2.leftTrigger() > ANALOG_PRESSED));

                    /*if (gamepadPlus2.leftTrigger() > ANALOG_PRESSED && glyphLimit.getState()) {
                        if(gamepadPlus2.rightBumper()){
                            lift.setPower(LIFT_POWER_DOWN*0.75);
                        }else{
                            lift.setPower(LIFT_POWER_DOWN);
                        }

                    } else if (gamepadPlus2.rightTrigger() > ANALOG_PRESSED && lift.getCurrentPosition() < 1450) {
                        if(gamepadPlus2.rightBumper()){
                            lift.setPower(LIFT_POWER_UP*0.75);
                        }else{
                            lift.setPower(LIFT_POWER_UP);
                        }

                    } else
                    if (gamepadPlus2.a()) {
                        if (!decrementPressed) {
                            if (lift.getCurrentPosition() < LIFT_POSITION1 + LIFT_GRACE_AREA) {
                                liftIncriment = 0;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION2 + LIFT_GRACE_AREA) {
                                liftIncriment = 1;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION3 + LIFT_GRACE_AREA) {
                                liftIncriment = 2;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION4 + LIFT_GRACE_AREA) {
                                liftIncriment = 3;
                            }
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            glyphLiftState = liftState.POSITION;
                        }
                        decrementPressed = true;
                    } else if (gamepadPlus2.y()) {
                        if (!incrementPressed) {
                            if (lift.getCurrentPosition() < LIFT_POSITION1 - LIFT_GRACE_AREA) {
                                liftIncriment = 1;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION2 - LIFT_GRACE_AREA) {
                                liftIncriment = 2;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION3 - LIFT_GRACE_AREA) {
                                liftIncriment = 3;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION4 - LIFT_GRACE_AREA) {
                                liftIncriment = 4;
                            }
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            glyphLiftState = liftState.POSITION;
                        }
                        incrementPressed = true;
                    }/* else {
                        lift.setPower(0);
                    }

                    break;
                case POSITION:
                    if (gamepadPlus2.a() && liftIncriment > 0) {
                        if (!decrementPressed) {
                            liftIncriment--;
                            decrementPressed = true;
                        }

                    } else {
                        decrementPressed = false;
                    }
                    if (gamepadPlus2.y() && liftIncriment < 4) {
                        if (!incrementPressed) {
                            liftIncriment++;
                            incrementPressed = true;
                        }

                    } else {
                        incrementPressed = false;
                    }
                    if (liftIncriment == 1) {
                        lift.setTargetPosition(LIFT_POSITION1);
                    } else if (liftIncriment == 2) {
                        lift.setTargetPosition(LIFT_POSITION2);
                    } else if (liftIncriment == 3) {
                        lift.setTargetPosition(LIFT_POSITION3);
                    } else if (liftIncriment == 4) {
                        lift.setTargetPosition(LIFT_POSITION4);
                    } else if (liftIncriment == 0) {
                        intake.setLiftPower(LIFT_POWER_DOWN);
                    } else if (liftIncriment == 1.5){
                        lift.setTargetPosition(ROTATE_POSITION);
                    }
                    if (!lift.isBusy() || gamepadPlus2.leftTrigger() > ANALOG_PRESSED || gamepadPlus2.rightTrigger() > ANALOG_PRESSED || !glyphLimit.getState()) {
                        glyphLiftState = liftState.MANUAL;
                        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    lift.setPower(LIFT_POWER_UP);
                    break;
            }*/

            //Drivetrain controls
            /*switch (hand){
                case LEFT:
                    thrust = -gamepad1.left_stick_y;
                    sideways = gamepad1.left_stick_x;
                    pivot = gamepad1.right_stick_x;
                    break;
                case RIGHT:
                    thrust = -gamepad1.right_stick_y;
                    sideways = gamepad1.right_stick_x;
                    pivot = gamepad1.left_stick_x;
                    break;
            }


            rfPower = thrust - sideways - pivot;
            rbPower = thrust + sideways - pivot;
            lfPower = thrust + sideways + pivot;
            lbPower = thrust - sideways + pivot;

            if (gamepadPlus1.rightTrigger() > 0.25) {
                rf.setPower(rfPower / 2);
                rb.setPower(rbPower / 2);
                lf.setPower(lfPower / 2);
                lb.setPower(lbPower / 2);
            } else {
                rf.setPower(rfPower*.75);
                rb.setPower(rbPower*.75);
                lf.setPower(lfPower*.75);
                lb.setPower(lbPower*.75);
            }*/
            switch (hand){
                case LEFT:
                    if(gamepadPlus1.rightTrigger() > ANALOG_PRESSED){
                        drive.moveNoIMU(gamepadPlus1.getAngleLeftStick(), gamepadPlus1.getDistanceFromCenterLeft()*0.5, true, -gamepadPlus1.rightStickX()*0.5);
                    }else{
                        drive.moveNoIMU(gamepadPlus1.getAngleLeftStick(), gamepadPlus1.getDistanceFromCenterLeft()*0.85, true, -gamepadPlus1.rightStickX()*0.85);
                    }
                    break;
                case RIGHT:
                    if(gamepadPlus1.rightTrigger() > ANALOG_PRESSED){
                        drive.moveNoIMU(gamepadPlus1.getAngleRightStick(), gamepadPlus1.getDistanceFromCenterRight()*0.5, true, -gamepadPlus1.leftStickX()*0.5);
                    }else{
                        drive.moveNoIMU(gamepadPlus1.getAngleRightStick(), gamepadPlus1.getDistanceFromCenterRight()*0.85, true, -gamepadPlus1.leftStickX()*0.85);
                    }
            }

            if(gamepadPlus2.leftTrigger() > ANALOG_PRESSED && glyphLimit.getState()){
                desiredEncoderPosition -= 100;
            }else if(gamepadPlus2.rightTrigger() > ANALOG_PRESSED && intake.returnLiftPosition() < 2600){
                desiredEncoderPosition += 100;
            }

            if(desiredEncoderPosition > 2600) { desiredEncoderPosition = 2600; }
            //else if(desiredEncoderPosition < 0) { desiredEncoderPosition = 0; }

            if(!(gamepadPlus2.leftTrigger() > ANALOG_PRESSED) && !(gamepadPlus2.rightTrigger() > ANALOG_PRESSED) && liftTime.seconds() > 2){
                desiredEncoderPosition = intake.returnLiftPosition();
            }

            if(!glyphLimit.getState()){
                if(!limitSwitchPressed) {
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    desiredEncoderPosition = 0;
                }
                limitSwitchPressed = true;
            }else{
                limitSwitchPressed = false;
            }

            if (gamepadPlus2.y() && intake.returnLiftPosition() < LIFT_POSITIONS[3]) {
                if (!incrementPressed) {
                    int currentPosition = intake.returnLiftPosition();
                    liftTime.reset();
                    if(currentPosition < LIFT_POSITIONS[0] - LIFT_GRACE_AREA){
                        desiredEncoderPosition = LIFT_POSITIONS[0];
                    }else if(currentPosition < LIFT_POSITIONS[1] - LIFT_GRACE_AREA){
                        desiredEncoderPosition = LIFT_POSITIONS[1];
                    }
                    else if(currentPosition < LIFT_POSITIONS[2] - LIFT_GRACE_AREA){
                        desiredEncoderPosition = LIFT_POSITIONS[2];
                    }else if(currentPosition < LIFT_POSITIONS[3] - LIFT_GRACE_AREA){
                        desiredEncoderPosition = LIFT_POSITIONS[3];
                    }
                    incrementPressed = true;
                }

            } else {
                incrementPressed = false;
            }
            if (gamepadPlus2.a() && intake.returnLiftPosition() > LIFT_POSITIONS[0]) {
                if (!decrementPressed) {
                    int currentPosition = intake.returnLiftPosition();
                    liftTime.reset();
                    if(currentPosition > LIFT_POSITIONS[0] + LIFT_GRACE_AREA && currentPosition < LIFT_POSITIONS[1] + LIFT_GRACE_AREA){
                        desiredEncoderPosition = LIFT_POSITIONS[0];
                    }else if(currentPosition > LIFT_POSITIONS[1] + LIFT_GRACE_AREA && currentPosition < LIFT_POSITIONS[2] + LIFT_GRACE_AREA){
                        desiredEncoderPosition = LIFT_POSITIONS[1];
                    }
                    else if(currentPosition > LIFT_POSITIONS[2] + LIFT_GRACE_AREA && currentPosition < LIFT_POSITIONS[3] + LIFT_GRACE_AREA){
                        desiredEncoderPosition = LIFT_POSITIONS[2];
                    }else if(currentPosition > LIFT_POSITIONS[3] + LIFT_GRACE_AREA){
                        desiredEncoderPosition = LIFT_POSITIONS[3];
                    }else if(currentPosition < LIFT_POSITIONS[0]){
                        desiredEncoderPosition = 0;
                    }
                    decrementPressed = true;
                }

            } else {
                decrementPressed = false;
            }


            /*lift.setTargetPosition(desiredEncoderPosition);
            lift.setPower(1);*/
            intake.setLiftTargetPosition(desiredEncoderPosition, LIFT_POWER_UP);

            //Relic Extension Motor Controls with Encoder Limits
            if (gamepadPlus2.rightBumper()) {
                relic.extend(-RELIC_ARM_EXTENSION_POWER, gamepad2.dpad_up && relic_extension.getCurrentPosition() > -2200);
            } else{
                relic.extend(-RELIC_ARM_EXTENSION_POWER*0.7, gamepad2.dpad_up && relic_extension.getCurrentPosition() > -2200);
            }

            if(gamepadPlus2.rightBumper()){
                relic.retract(-RELIC_ARM_RETRACTION_POWER, gamepad2.dpad_down && relic_extension.getCurrentPosition() < -200);
            }else{
                relic.retract(-RELIC_ARM_RETRACTION_POWER*0.7, gamepad2.dpad_down && relic_extension.getCurrentPosition() < -200);
            }

            if(!gamepadPlus2.dpadUp() && !gamepadPlus2.dpadDown()){
                relic.extensionPowerZero();
            }

            telemetry.addData("relic extension position", relic_extension.getCurrentPosition());

            //Claw servo controls
            if (gamepad2.dpad_left) {
                relic.pickUpRelic();
            } else if (gamepad2.dpad_right) {
                relic.releaseRelic();
            }
            if(gamepadPlus2.x()){
                relic.setArmPosition(RELIC_ARM_GRAB_POS);
            }else if(gamepadPlus1.a()){
                relic.setArmPosition(RELIC_ARM_ORIGIN);
                relic.pickUpRelic();
                relic.setTiltPosition(1);
            }
            if(gamepadPlus2.b()){
                relic.setTiltPosition(0.6);
            }
            //Relic Arm Servo Controls
            if (relic.returnArmPos()< .4) {
                relic.adjustArm((-gamepad2.right_stick_y > 0.1 && relic.returnArmPos() <= 1), 0.05);
                relic.adjustArm((-gamepad2.right_stick_y < -0.1 && relic.returnArmPos() >= 0.04), -0.05);

            } else {
                relic.adjustArm(-gamepad2.right_stick_y > 0.1 && relic.returnArmPos() <= 1, .005);
                relic.adjustArm(-gamepad2.right_stick_y < -0.1 && relic.returnArmPos() >= 0.04, -.005);
            }

            //Relic Tilt Servo Controls
            relic.tiltRelic(-gamepad2.left_stick_y > 0.1 && relic.returnTiltPos() <= 0.9, 0.01);
            relic.tiltRelic(-gamepad2.left_stick_y < -0.1 && relic.returnTiltPos() >= 0.01, -0.01);
            //Intake Toggle
            if (gamepadPlus1.rightBumper()) {
                intake.dispenseGlyph();
                intakeDirection = false;

            } else if (!intakePressed && gamepadPlus1.leftBumper()) {
                intakePressed = true;
                intakeDirection = !intakeDirection;
                if (intakeDirection) {
                    intake.secureGlyph();

                } else {
                    intake.setIntakePowerZero();

                }
            } else if (!gamepadPlus1.leftBumper()) {
                if (intakeDirection) {
                    intake.secureGlyph();

                } else {
                    intake.setIntakePowerZero();

                }
                intakePressed = false;
            }
            telemetry.addData("intake direction", intakeDirection);
            telemetry.addData("intake pressed", intakePressed);
            telemetry.addData("Glyph Height", lift.getCurrentPosition());
            telemetry.addData("Relic Tilt Pos", relic_tilt.getPosition());
            telemetry.addData("Relic Arm Pos", relic_arm.getPosition());
            telemetry.addData("Glyph State", glyphLiftState);
            telemetry.addData("Desired Encoder Position", desiredEncoderPosition);
            telemetry.addData("Limit Switch Pressed", limitSwitchPressed);
            telemetry.addData("Glyph Limit", glyphLimit.getState());
            telemetry.update();

        }
        relic.releaseRelic();
    }
}