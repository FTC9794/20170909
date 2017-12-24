package org.firstinspires.ftc.teamcode.Teleop;

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
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.OmniDirectionalDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Jewel.TwoPointJewelArm;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;

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

    double thrust, sideways, pivot, rfPower, rbPower, lfPower, lbPower;

    ClawThreePoint relic;
    DualWheelIntake intake;
    OmniDirectionalDrive drive;
    TwoPointJewelArm jewel;

    List<DcMotor> driveMotors;

    boolean spinAtOrigin = true;
    boolean spinPressed = false;
    boolean rightBumperPressed = false;
    boolean leftBumperPressed = false;
    boolean lowerLift = false;
    boolean intakeDirection = false;
    boolean intakePressed = false;


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

    final double ANALOG_PRESSED = .5;

    final double LIFT_GRACE_AREA = 100;

    final double LIFT_POWER_UP = 1;
    final double LIFT_POWER_DOWN = -1;

    final double JEWEL_PAN_START = .55;
    final double JEWEL_TILT_START = 1;

    final double ROTATION_TIME = 50;



    final double RELIC_ARM_ORIGIN = 0;
    final double RELIC_ARM_GRAB_POS = .86;

    final double RELIC_ARM_EXTENSION_POWER = 1;
    final double RELIC_ARM_RETRACTION_POWER = -1;

    Handiness hand = Handiness.LEFT;
    boolean selectedHand = false;

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

        relic_arm = hardwareMap.servo.get("relic_arm");
        relic_claw = hardwareMap.servo.get("relic_claw");
        relic_tilt = hardwareMap.servo.get("relic_tilt");

        rightWheel1 = hardwareMap.crservo.get("right_glyph1");
        leftWheel1 = hardwareMap.crservo.get("left_glyph1");
        rightWheel2 = hardwareMap.crservo.get("right_glyph2");
        leftWheel2 = hardwareMap.crservo.get("left_glyph2");
        //leftWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        glyphLiftState = liftState.MANUAL;
        glyphRotateState = rotateState.MANUAL;

        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relic_extension.setDirection(DcMotorSimple.Direction.REVERSE);

        rotateTime = new ElapsedTime();
        gamepadPlus1 = new GamepadPlus(gamepad1);
        gamepadPlus2 = new GamepadPlus(gamepad2);



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

        while (opModeIsActive()) {

            //Glyph rotation state machine
            switch(glyphRotateState){
                case MANUAL:
                    if(gamepadPlus2.leftBumper()){
                        if(!spinPressed){
                            if(lift.getCurrentPosition()>ROTATE_POSITION) {
                                glyphRotateState = rotateState.ROTATING;
                            }
                        }
                        spinPressed = true;
                    }else{
                        spinPressed = false;
                    }
                    break;
                case LIFTING:

                    if(gamepadPlus2.leftTrigger()>ANALOG_PRESSED||gamepadPlus2.rightTrigger()>ANALOG_PRESSED) {
                        glyphRotateState = rotateState.MANUAL;
                    }else if(!lift.isBusy()){
                        glyphRotateState = rotateState.ROTATING;
                        rotateTime.reset();
                    }
                    break;
                case ROTATING:
                    intake.spin();
                    if(rotateTime.milliseconds()>ROTATION_TIME){

                        spinAtOrigin = !spinAtOrigin;
                        if(lowerLift){
                            glyphRotateState = rotateState.LOWERING;
                            glyphLiftState = liftState.POSITION;
                            liftIncriment = 0;
                            //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        }else{
                            glyphRotateState = rotateState.MANUAL;
                        }
                    }
                    break;
                case LOWERING:
                    if(!lift.isBusy()){
                        glyphRotateState = rotateState.MANUAL;
                    }
                    break;
            }

            //Glyph Lift State Machine
            switch (glyphLiftState) {
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

                    } else if (gamepadPlus2.a()) {
                        if (!leftBumperPressed) {
                            if (lift.getCurrentPosition() < LIFT_POSITION1 + LIFT_GRACE_AREA) {
                                liftIncriment = 0;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION2 + LIFT_GRACE_AREA) {
                                liftIncriment = 1;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION3 + LIFT_GRACE_AREA) {
                                liftIncriment = 2;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION4 + LIFT_GRACE_AREA) {
                                liftIncriment = 3;
                            }
                            //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            //glyphLiftState = TheWizardTeleop.liftState.POSITION;
                        }
                        leftBumperPressed = true;
                    } else if (gamepadPlus2.y()) {
                        if (!rightBumperPressed) {
                            if (lift.getCurrentPosition() < LIFT_POSITION1 - LIFT_GRACE_AREA) {
                                liftIncriment = 1;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION2 - LIFT_GRACE_AREA) {
                                liftIncriment = 2;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION3 - LIFT_GRACE_AREA) {
                                liftIncriment = 3;
                            } else if (lift.getCurrentPosition() < LIFT_POSITION4 - LIFT_GRACE_AREA) {
                                liftIncriment = 4;
                            }
                            //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            //glyphLiftState = TheWizardTeleop.liftState.POSITION;
                        }
                        rightBumperPressed = true;
                    } else {
                        lift.setPower(0);
                    }*/

                    break;
                case POSITION:
                    /*if (gamepadPlus2.a() && liftIncriment > 0) {
                        if (!leftBumperPressed) {
                            liftIncriment--;
                            leftBumperPressed = true;
                        }

                    } else {
                        leftBumperPressed = false;
                    }
                    if (gamepadPlus2.y() && liftIncriment < 4) {
                        if (!rightBumperPressed) {
                            liftIncriment++;
                            rightBumperPressed = true;
                        }

                    } else {
                        rightBumperPressed = false;
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
                        lift.setTargetPosition(0);
                    } else if (liftIncriment == 1.5){
                        lift.setTargetPosition(ROTATE_POSITION);
                    }
                    if (!lift.isBusy() || gamepadPlus2.leftTrigger() > ANALOG_PRESSED || gamepadPlus2.rightTrigger() > ANALOG_PRESSED) {
                        glyphLiftState = liftState.MANUAL;
                        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    lift.setPower(LIFT_POWER_UP);*/
                    break;
            }

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
            if(gamepadPlus2.y()){
                relic.setArmPosition(RELIC_ARM_GRAB_POS);
            }else if(gamepadPlus1.a() || gamepadPlus2.a()){
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
            telemetry.update();

        }
        relic.releaseRelic();
    }
}