package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.GamepadPlus;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.OmniDirectionalDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.FourArmRotatingGlyph;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.IGlyph;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;

import java.io.File;
import java.util.ArrayList;

/**
 * Created by Sarthak on 11/1/2017.
 */
@TeleOp(name = "The Wizard Teleop", group = "Teleop")
public class TheWizardTeleop extends LinearOpMode {
    ElapsedTime rotateTime;

    GamepadPlus gamepadPlus1;
    GamepadPlus gamepadPlus2;

    Servo right1, right2, left1, left2, spin;
    Servo pan, tilt;
    Servo relic_claw, relic_arm, relic_tilt;
    CRServo rightWheel1, leftWheel1;
    DigitalChannel glyphLimit;
    DcMotor lift, relic_extension;
    DcMotor rf, rb, lf, lb;
    double thrust, sideways, pivot, rfPower, rbPower, lfPower, lbPower;

    FourArmRotatingGlyph glyph;
    OmniDirectionalDrive drive;
    IIMU imu;
    ClawThreePoint relic;
    BNO055IMU boschIMU;

    ArrayList<DcMotor> driveMotors;

    boolean topGripOpen = true;
    boolean bottomGripOpen = true;
    boolean spinAtOrigin = true;
    boolean spinPressed = false;
    boolean gripPressed1 = false;
    boolean gripPressed2 = false;
    boolean rightBumperPressed = false;
    boolean leftBumperPressed = false;
    boolean lowerLift = false;
    boolean resetPressed = false;
    boolean intakeDirection = false;
    boolean intakePressed = false;
    boolean outTakeOn = false;

    enum liftState{
        MANUAL,
        POSITION
    }
    liftState glyphLiftState;

    enum rotateState{
        MANUAL,
        LIFTING,
        ROTATING,
        LOWERING
    }
    rotateState glyphRotateState;

    double liftIncriment = 0;


    final double GRIP_OPEN1 = .5;
    final double GRIP_OPEN2 = .5;
    final double GRIP_CLOSE1 = 0;
    final double GRIP_CLOSE2 = 0;

    final double SPIN_START = 0;
    final double SPIN_ROTATED = .95;

    final int LIFT_POSITION1 = 1100;
    final int LIFT_POSITION2 = 2500;
    final int LIFT_POSITION3 = 4250;
    final int LIFT_POSITION4 = 6400;

    final double ANALOG_PRESSED = .5;

    final double LIFT_GRACE_AREA = 100;

    final double LIFT_POWER_UP = 1;
    final double LIFT_POWER_DOWN = -1;

    final double JEWEL_PAN_START = .5;
    final double JEWEL_TILT_START = 1;

    final double ROTATION_TIME = 1000;

    final double RELIC_CLAW_CLOSED = 1;
    final double RELIC_CLAW_OPENED = 0;

    final double RELIC_TILT_ORIGIN = 1;

    final double RELIC_ARM_ORIGIN = 0;
    final double RELIC_ARM_GRAB_POS = 0.96;

    final double RELIC_ARM_EXTENSION_POWER = 1;
    final double RELIC_ARM_RETRACTION_POWER = -1;


    @Override
    public void runOpMode() throws InterruptedException {
        //right1 = hardwareMap.servo.get("right_glyph1");
        right2 = hardwareMap.servo.get("right_glyph2");
        //left1 = hardwareMap.servo.get("left_glyph1");
        left2 = hardwareMap.servo.get("left_glyph2");
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
        rightWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftWheel1 = hardwareMap.crservo.get("left_glyph1");

        telemetry.addData("Init", "IMU Calibrating");
        telemetry.update();
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.setOffset(0);
        telemetry.addData("Init", "IMU Instantiated");
        telemetry.update();

        drive = new OmniDirectionalDrive(driveMotors, imu, telemetry);
        telemetry.addData("Init", "Drive and IMU Created");
        telemetry.update();

        glyphLiftState = liftState.MANUAL;
        glyphRotateState = rotateState.MANUAL;

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right2.setDirection(Servo.Direction.REVERSE);
        //left1.setDirection(Servo.Direction.REVERSE);

        //right1.setPosition(GRIP_OPEN1);
        right2.setPosition(GRIP_OPEN2);
        //left1.setPosition(GRIP_OPEN1);
        left2.setPosition(GRIP_OPEN2);

        spin.setPosition(SPIN_START);

        pan.setPosition(JEWEL_PAN_START);
        tilt.setPosition(JEWEL_TILT_START);

        relic_claw.setPosition(RELIC_CLAW_CLOSED);
        relic_tilt.setPosition(RELIC_TILT_ORIGIN);
        relic_arm.setPosition(RELIC_ARM_ORIGIN);

        //glyph = new FourArmRotatingGlyph(right1, right2, left1, left2, spin, lift);
        rotateTime = new ElapsedTime();
        gamepadPlus1 = new GamepadPlus(gamepad1);
        gamepadPlus2 = new GamepadPlus(gamepad2);
        relic = new ClawThreePoint(relic_extension, relic_arm, relic_tilt, relic_claw, telemetry);
        telemetry.addData("Initialized", "Done");
        telemetry.update();
        imu.setOffset(180);
        waitForStart();

        while(opModeIsActive()){

            //Glyph rotation state machine
            /*switch(glyphRotateState){
                case MANUAL:
                    if(gamepadPlus2.x() || gamepadPlus1.dpadDown()){
                        if(spinAtOrigin){
                            if(!gripPressed2){
                                if(bottomGripOpen){
                                    glyph.secureGlyph();
                                    bottomGripOpen = false;
                                }else{
                                    glyph.dispenseGlyph();
                                    bottomGripOpen = true;
                                }
                            }
                        }else{
                            if(!gripPressed2){
                                if(topGripOpen){
                                    glyph.secureTopGlyph();
                                    topGripOpen = false;
                                }else{
                                    glyph.dispenseTopGlyph();
                                    topGripOpen = true;
                                }
                            }
                        }

                        gripPressed2 = true;
                    }else{
                        gripPressed2 = false;
                    }
                    if(gamepadPlus2.b() || gamepadPlus1.dpadUp()){
                        if(spinAtOrigin){
                            if(!gripPressed1){
                                if(topGripOpen){
                                    glyph.secureTopGlyph();

                                    topGripOpen = false;
                                }else{
                                    glyph.dispenseTopGlyph();
                                    topGripOpen = true;
                                }
                            }
                        }else{
                            if(!gripPressed1){
                                if(bottomGripOpen){
                                    glyph.secureGlyph();
                                    bottomGripOpen = false;
                                }else{
                                    glyph.dispenseGlyph();
                                    bottomGripOpen = true;
                                }
                            }
                        }

                        gripPressed1 = true;
                    }else{
                        gripPressed1 = false;
                    }
                    if(gamepadPlus2.a()){
                        if(!spinPressed){
                            if(lift.getCurrentPosition()>LIFT_POSITION1){
                                glyphRotateState = rotateState.ROTATING;
                                rotateTime.reset();

                                lowerLift = false;
                            }else{
                                glyphLiftState = liftState.POSITION;
                                liftIncriment = 1;
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
                    glyph.rotate();
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
            }*/
            //Glyph Lift State Machine
            switch(glyphLiftState){
                case MANUAL:
                    if(!glyphLimit.getState()){
                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    }
                    if(gamepadPlus2.leftTrigger()>ANALOG_PRESSED && glyphLimit.getState()){
                        lift.setPower(LIFT_POWER_DOWN);
                    }else if(gamepadPlus2.rightTrigger()>ANALOG_PRESSED && lift.getCurrentPosition()<LIFT_POSITION4){
                        lift.setPower(LIFT_POWER_UP);
                    }else if(gamepadPlus2.leftBumper()){
                        if(!leftBumperPressed){
                            if(lift.getCurrentPosition()<LIFT_POSITION1+LIFT_GRACE_AREA){
                                liftIncriment = 0;
                            }else if(lift.getCurrentPosition()<LIFT_POSITION2+LIFT_GRACE_AREA){
                                liftIncriment = 1;
                            }else if(lift.getCurrentPosition()<LIFT_POSITION3+LIFT_GRACE_AREA){
                                liftIncriment = 2;
                            }else if(lift.getCurrentPosition()<LIFT_POSITION4+LIFT_GRACE_AREA){
                                liftIncriment = 3;
                            }
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            glyphLiftState = liftState.POSITION;
                        }
                        leftBumperPressed = true;
                    }else if(gamepadPlus2.rightBumper()){
                        if(!rightBumperPressed){
                            if(lift.getCurrentPosition()<LIFT_POSITION1-LIFT_GRACE_AREA){
                                liftIncriment = 1;
                            }else if(lift.getCurrentPosition()<LIFT_POSITION2-LIFT_GRACE_AREA){
                                liftIncriment = 2;
                            }else if(lift.getCurrentPosition()<LIFT_POSITION3-LIFT_GRACE_AREA){
                                liftIncriment = 3;
                            }else if(lift.getCurrentPosition()<LIFT_POSITION4-LIFT_GRACE_AREA){
                                liftIncriment = 4;
                            }
                            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            glyphLiftState = liftState.POSITION;
                        }
                        rightBumperPressed = true;
                    }else{
                        lift.setPower(0);
                    }

                    break;
                case POSITION:
                    if(gamepadPlus2.leftBumper()&&liftIncriment>0){
                        if(!leftBumperPressed){
                            liftIncriment--;
                            leftBumperPressed = true;
                        }

                    }else{
                        leftBumperPressed = false;
                    }
                    if(gamepadPlus2.rightBumper()&&liftIncriment<4){
                        if(!rightBumperPressed){
                            liftIncriment++;
                            rightBumperPressed = true;
                        }

                    }else{
                        rightBumperPressed=false;
                    }
                    if(liftIncriment==1){
                        lift.setTargetPosition(LIFT_POSITION1);
                    }else if(liftIncriment==2){
                        lift.setTargetPosition(LIFT_POSITION2);
                    }else if(liftIncriment==3){
                        lift.setTargetPosition(LIFT_POSITION3);
                    }else if(liftIncriment==4){
                        lift.setTargetPosition(LIFT_POSITION4);
                    }else if(liftIncriment==0){
                        lift.setTargetPosition(0);
                    }
                    if(!lift.isBusy()||gamepadPlus2.leftTrigger()>ANALOG_PRESSED||gamepadPlus2.rightTrigger()>ANALOG_PRESSED){
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

            if(gamepad1.right_bumper){
                rf.setPower(rfPower/3);
                rb.setPower(rbPower/3);
                lf.setPower(lfPower/3);
                lb.setPower(lbPower/3);
            }else {
                rf.setPower(rfPower);
                rb.setPower(rbPower);
                lf.setPower(lfPower);
                lb.setPower(lbPower);
            }


            //Relic Extension Motor Controls with Encoder Limits
            if(gamepad2.dpad_up && relic_extension.getCurrentPosition() < 12200){
                relic_extension.setPower(RELIC_ARM_EXTENSION_POWER);
            }else if(gamepad2.dpad_down && relic_extension.getCurrentPosition() > 150){
                relic_extension.setPower(RELIC_ARM_RETRACTION_POWER);
            }else{
                relic_extension.setPower(0);
            }

            //Claw servo controls
            if(gamepad2.dpad_left){
                relic.pickUpRelic();
            }else if(gamepad2.dpad_right){
                relic.releaseRelic();
            }

            //Relic Arm Servo Controls
            if(relic_arm.getPosition() < 0.65) {
                if (-gamepad2.right_stick_y > 0.1 && relic_arm.getPosition() <= RELIC_ARM_GRAB_POS) {
                    relic.adjustArm(true, 0, 1, 0.05);
                } else if (-gamepad2.right_stick_y < -0.1 && relic_arm.getPosition() >= 0.04) {
                    relic.adjustArm(true, 0, 1, -0.05);
                }
            }else{
                if (-gamepad2.right_stick_y > 0.1 && relic_arm.getPosition() <= RELIC_ARM_GRAB_POS) {
                    relic.adjustArm(true, 0, 1, 0.02);
                } else if (-gamepad2.right_stick_y < -0.1 && relic_arm.getPosition() >= 0.04) {
                    relic.adjustArm(true, 0, 1, -0.02);
                }
            }

            //Relic Tilt Servo Controls
            if(-gamepad2.left_stick_y > 0.1 && relic_tilt.getPosition() <= 0.96){
                relic.tiltRelic(true, 0, 1, 0.04);
            }else if(-gamepad2.left_stick_y < -0.1 && relic_tilt.getPosition() >= 0.04){
                relic.tiltRelic(true, 0, 1, -0.04);
            }
            telemetry.addData("RelicArmAngle", relic.returnArmAngle());
            telemetry.addData("RelicTiltPos", relic.returnTiltPos());
            telemetry.addData("ArmPos", relic_arm.getPosition());
            telemetry.addData("RelicTilt", relic_tilt.getPosition());
            telemetry.addData("RelicExtensionEncoders", relic_extension.getCurrentPosition());

            //Duel Wheel Intake
            /*f(gamepadPlus2.x()) {
                if (!intakePressed) {
                    intakePressed = true;
                    if (intakeDirection) {
                        rightWheel1.setPower(-1);
                        leftWheel1.setPower(-1);
                        intakeDirection = true;
                    } else {
                        rightWheel1.setPower(1);
                        ;
                        leftWheel1.setPower(1);
                        intakeDirection = false;
                    }
                }
            }else{
                intakePressed=false;
            }

            if(gamepadPlus2.y()){
                rightWheel1.setPower(0);;
                leftWheel1.setPower(0);
                intakeDirection = false;
            }*/
            if(gamepadPlus2.x()){
                rightWheel1.setPower(-1);
                leftWheel1.setPower(-1);
            }else if (gamepadPlus2.b()){
                rightWheel1.setPower(1);
                leftWheel1.setPower(1);
            }else{
                rightWheel1.setPower(0);
                leftWheel1.setPower(0);
            }


            //Telemetry
            telemetry.addData("relic_extension encoders", relic_extension.getCurrentPosition());

            if(gamepad1.left_bumper&&!resetPressed){
                imu.setAsZero();
                resetPressed = true;
                telemetry.addData("IMU", "reset");
            }else{
                resetPressed = false;
            }
            /*if(gamepad1.right_bumper) {
                drive.move(1, 0, gamepadPlus1.getDistanceFromCenterLeft(), .4, gamepadPlus1.getAngleLeftStick(), .02/3, 0, imu.getZAngle() + gamepadPlus1.rightStickX() * 40, false, 0);
            }else{
                drive.move(1, 0, gamepadPlus1.getDistanceFromCenterLeft(), 1, gamepadPlus1.getAngleLeftStick(), .02, 0, imu.getZAngle() + gamepadPlus1.rightStickX() * 40, false, 0);
            }*/
            telemetry.update();
        }


    }
}
