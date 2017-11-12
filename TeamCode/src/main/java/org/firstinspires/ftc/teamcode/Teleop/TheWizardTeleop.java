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
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;
import java.util.ArrayList;

/**
 * Created by Sarthak on 11/1/2017.
 */
@TeleOp(name = "The Wizard Teleop", group = "Teleop")
public class TheWizardTeleop extends LinearOpMode {
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

    ArrayList<DcMotor> driveMotors;

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


    final double SPIN_START = .05;
    final double SPIN_ROTATED = .95;

    final int LIFT_POSITION1 = 1100;
    final int LIFT_POSITION2 = 3200;
    final int LIFT_POSITION3 = 5300;
    final int LIFT_POSITION4 = 6900;
    final int ROTATE_POSITION = 2000;

    final double ANALOG_PRESSED = .5;

    final double LIFT_GRACE_AREA = 100;

    final double LIFT_POWER_UP = 1;
    final double LIFT_POWER_DOWN = -1;

    final double JEWEL_PAN_START = .5;
    final double JEWEL_TILT_START = 1;

    final double ROTATION_TIME = 1250;

    final double RELIC_CLAW_CLOSED = 1;
    final double RELIC_CLAW_OPENED = 0;

    final double RELIC_TILT_ORIGIN = 1;

    final double RELIC_ARM_ORIGIN = 0;
    final double RELIC_ARM_GRAB_POS = 0.96;

    final double RELIC_ARM_EXTENSION_POWER = 1;
    final double RELIC_ARM_RETRACTION_POWER = -1;


    @Override
    public void runOpMode() throws InterruptedException {

        spin = hardwareMap.servo.get("spin_grip");

        pan = hardwareMap.servo.get("jewel_pan");
        tilt = hardwareMap.servo.get("jewel_tilt");

        lift = hardwareMap.dcMotor.get("glyph_lift");

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
        leftWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        glyphLiftState = liftState.MANUAL;
        glyphRotateState = rotateState.MANUAL;

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spin.setPosition(SPIN_START);

        pan.setPosition(JEWEL_PAN_START);
        tilt.setPosition(JEWEL_TILT_START);

        relic_claw.setPosition(RELIC_CLAW_CLOSED);
        relic_tilt.setPosition(RELIC_TILT_ORIGIN);
        relic_arm.setPosition(RELIC_ARM_ORIGIN);

        rotateTime = new ElapsedTime();
        gamepadPlus1 = new GamepadPlus(gamepad1);
        gamepadPlus2 = new GamepadPlus(gamepad2);
        relic = new ClawThreePoint(relic_extension, relic_arm, relic_tilt, relic_claw, telemetry);
        telemetry.addData("Initialized", "Done");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            //Glyph rotation state machine
            switch(glyphRotateState){
                case MANUAL:
                    if(gamepadPlus2.a()){
                        if(!spinPressed){
                            if(lift.getCurrentPosition()>ROTATE_POSITION){
                                glyphRotateState = rotateState.ROTATING;
                                rotateTime.reset();

                                lowerLift = false;
                            }else{
                                glyphLiftState = liftState.POSITION;
                                liftIncriment = 1.5;
                                glyphRotateState = rotateState.LIFTING;
                                lowerLift = true;
                                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                    if(spinAtOrigin){
                        spin.setPosition(SPIN_ROTATED);
                    }else{
                        spin.setPosition(SPIN_START);
                    }
                    if(rotateTime.milliseconds()>ROTATION_TIME){

                        spinAtOrigin = !spinAtOrigin;
                        if(lowerLift){
                            glyphRotateState = rotateState.LOWERING;
                            glyphLiftState = liftState.POSITION;
                            liftIncriment = 0;
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
                    if (!glyphLimit.getState()) {
                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    }
                    if (gamepadPlus2.leftTrigger() > ANALOG_PRESSED && glyphLimit.getState()) {
                        lift.setPower(LIFT_POWER_DOWN);
                    } else if (gamepadPlus2.rightTrigger() > ANALOG_PRESSED && lift.getCurrentPosition() < LIFT_POSITION4) {
                        lift.setPower(LIFT_POWER_UP);
                    } else if (gamepadPlus2.leftBumper()) {
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
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            glyphLiftState = liftState.POSITION;
                        }
                        leftBumperPressed = true;
                    } else if (gamepadPlus2.rightBumper()) {
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
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            glyphLiftState = liftState.POSITION;
                        }
                        rightBumperPressed = true;
                    } else {
                        lift.setPower(0);
                    }

                    break;
                case POSITION:
                    if (gamepadPlus2.leftBumper() && liftIncriment > 0) {
                        if (!leftBumperPressed) {
                            liftIncriment--;
                            leftBumperPressed = true;
                        }

                    } else {
                        leftBumperPressed = false;
                    }
                    if (gamepadPlus2.rightBumper() && liftIncriment < 4) {
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
                    lift.setPower(LIFT_POWER_UP);
                    break;
            }

            //Drivetrain controls
            thrust = -gamepad1.left_stick_y;
            sideways = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;

            rfPower = thrust - sideways - pivot;
            rbPower = thrust + sideways - pivot;
            lfPower = thrust + sideways + pivot;
            lbPower = thrust - sideways + pivot;

            if (gamepad1.right_bumper) {
                rf.setPower(rfPower / 3);
                rb.setPower(rbPower / 3);
                lf.setPower(lfPower / 3);
                lb.setPower(lbPower / 3);
            } else {
                rf.setPower(rfPower);
                rb.setPower(rbPower);
                lf.setPower(lfPower);
                lb.setPower(lbPower);
            }


            //Relic Extension Motor Controls with Encoder Limits
            if (gamepad2.dpad_up && relic_extension.getCurrentPosition() < 12200) {
                relic_extension.setPower(RELIC_ARM_EXTENSION_POWER);
            } else if (gamepad2.dpad_down && relic_extension.getCurrentPosition() > 150) {
                relic_extension.setPower(RELIC_ARM_RETRACTION_POWER);
            } else {
                relic_extension.setPower(0);
            }
            telemetry.addData("relic extension position", relic_extension.getCurrentPosition());

            //Claw servo controls
            if (gamepad2.dpad_left) {
                relic.pickUpRelic();
            } else if (gamepad2.dpad_right) {
                relic.releaseRelic();
            }

            //Relic Arm Servo Controls
            if (relic_arm.getPosition() < 0.65) {
                if (-gamepad2.right_stick_y > 0.1 && relic_arm.getPosition() <= RELIC_ARM_GRAB_POS) {
                    relic.adjustArm(true, 0, 1, 0.05);
                } else if (-gamepad2.right_stick_y < -0.1 && relic_arm.getPosition() >= 0.04) {
                    relic.adjustArm(true, 0, 1, -0.05);
                }
            } else {
                if (-gamepad2.right_stick_y > 0.1 && relic_arm.getPosition() <= RELIC_ARM_GRAB_POS) {
                    relic.adjustArm(true, 0, 1, 0.02);
                } else if (-gamepad2.right_stick_y < -0.1 && relic_arm.getPosition() >= 0.04) {
                    relic.adjustArm(true, 0, 1, -0.02);
                }
            }

            //Relic Tilt Servo Controls
            if (-gamepad2.left_stick_y > 0.1 && relic_tilt.getPosition() <= 0.96) {
                relic.tiltRelic(true, 0, 1, 0.04);
            } else if (-gamepad2.left_stick_y < -0.1 && relic_tilt.getPosition() >= 0.04) {
                relic.tiltRelic(true, 0, 1, -0.04);
            }

            //Intake Toggle
            if (gamepadPlus2.b()) {
                rightWheel1.setPower(-1);
                leftWheel1.setPower(-1);
                rightWheel2.setPower(-1);
                leftWheel2.setPower(-1);

            } else if (!intakePressed && gamepadPlus2.x()) {
                intakePressed = true;
                intakeDirection = !intakeDirection;
                if (intakeDirection) {
                    rightWheel1.setPower(1);
                    leftWheel1.setPower(1);
                    rightWheel2.setPower(1);
                    leftWheel2.setPower(1);

                } else {
                    rightWheel1.setPower(0);
                    leftWheel1.setPower(0);
                    rightWheel2.setPower(0);
                    leftWheel2.setPower(0);

                }
            } else if (!gamepadPlus2.x()) {
                if (intakeDirection) {
                    rightWheel1.setPower(1);
                    leftWheel1.setPower(1);
                    rightWheel2.setPower(1);
                    leftWheel2.setPower(1);

                } else {
                    rightWheel1.setPower(0);
                    leftWheel1.setPower(0);
                    rightWheel2.setPower(0);
                    leftWheel2.setPower(0);

                }
                intakePressed = false;
            }
            telemetry.addData("intake direction", intakeDirection);
            telemetry.addData("intake pressed", intakePressed);
            telemetry.addData("Glyph Height", lift.getCurrentPosition());
            telemetry.update();

        }
    }
}