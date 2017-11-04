package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    DigitalChannel glyphLimit;
    DcMotor lift, relic_extension;
    DcMotor rf, rb, lf, lb;
    double thrust, sideways, pivot, rfPower, rbPower, lfPower, lbPower;

    FourArmRotatingGlyph glyph;
    OmniDirectionalDrive drive;
    IIMU imu;
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

    final int LIFT_POSITION1 = 800;
    final int LIFT_POSITION2 = 2500;
    final int LIFT_POSITION3 = 4250;
    final int LIFT_POSITION4 = 6000;

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
    final double RELIC_ARM_GRAB_POS = 0.95;

    final double RELIC_ARM_EXTENSION_POWER = 1;
    final double RELIC_ARM_RETRACTION_POWER = -1;


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

        glyphLimit = hardwareMap.digitalChannel.get("glyph_limit");

        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

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

        /*telemetry.addData("Init", "IMU Calibrating");
        telemetry.update();
        //Initialize BOSCH IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        boschIMU.initialize(parameters);
        imu = new BoschIMU(boschIMU);

        imu.setOffset(0);
        telemetry.addData("Init", "IMU Instantiated");
        telemetry.update();
*/
        drive = new OmniDirectionalDrive(driveMotors, telemetry);
        telemetry.addData("Init", "Drive and IMU Created");
        telemetry.update();

        glyphLiftState = liftState.MANUAL;
        glyphRotateState = rotateState.MANUAL;

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right2.setDirection(Servo.Direction.REVERSE);
        left1.setDirection(Servo.Direction.REVERSE);

        right1.setPosition(GRIP_OPEN1);
        right2.setPosition(GRIP_OPEN2);
        left1.setPosition(GRIP_OPEN1);
        left2.setPosition(GRIP_OPEN2);

        spin.setPosition(SPIN_START);

        pan.setPosition(JEWEL_PAN_START);
        tilt.setPosition(JEWEL_TILT_START);

        relic_claw.setPosition(RELIC_CLAW_CLOSED);
        relic_tilt.setPosition(RELIC_TILT_ORIGIN);
        relic_arm.setPosition(RELIC_ARM_ORIGIN);

        glyph = new FourArmRotatingGlyph(right1, right2, left1, left2, spin, lift);
        rotateTime = new ElapsedTime();
        gamepadPlus1 = new GamepadPlus(gamepad1);
        gamepadPlus2 = new GamepadPlus(gamepad2);
        telemetry.addData("Initialized", "Done");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){


            switch(glyphRotateState){
                case MANUAL:
                    if(gamepadPlus2.x()){
                        if(spinAtOrigin){
                            if(!gripPressed2){
                                if(bottomGripOpen){
                                    right2.setPosition(GRIP_CLOSE2);
                                    left2.setPosition(GRIP_CLOSE2);
                                    bottomGripOpen = false;
                                }else{
                                    right2.setPosition(GRIP_OPEN2);
                                    left2.setPosition(GRIP_OPEN2);
                                    bottomGripOpen = true;
                                }
                            }
                        }else{
                            if(!gripPressed2){
                                if(topGripOpen){
                                    right1.setPosition(GRIP_CLOSE1);
                                    left1.setPosition(GRIP_CLOSE1);

                                    topGripOpen = false;
                                }else{
                                    right1.setPosition(GRIP_OPEN1);
                                    left1.setPosition(GRIP_OPEN1);
                                    topGripOpen = true;
                                }
                            }
                        }

                        gripPressed2 = true;
                    }else{
                        gripPressed2 = false;
                    }
                    if(gamepadPlus2.b()){
                        if(spinAtOrigin){
                            if(!gripPressed1){
                                if(topGripOpen){
                                    right1.setPosition(GRIP_CLOSE1);
                                    left1.setPosition(GRIP_CLOSE1);

                                    topGripOpen = false;
                                }else{
                                    right1.setPosition(GRIP_OPEN1);
                                    left1.setPosition(GRIP_OPEN1);
                                    topGripOpen = true;
                                }
                            }
                        }else{
                            if(!gripPressed1){
                                if(bottomGripOpen){
                                    right2.setPosition(GRIP_CLOSE2);
                                    left2.setPosition(GRIP_CLOSE2);
                                    bottomGripOpen = false;
                                }else{
                                    right2.setPosition(GRIP_OPEN2);
                                    left2.setPosition(GRIP_OPEN2);
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
                    if(spinAtOrigin) {
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


            thrust = -gamepad1.right_stick_y;
            sideways = gamepad1.right_stick_x;
            pivot = gamepad1.left_stick_x;

            rfPower = thrust + sideways + pivot;
            rbPower = thrust - sideways + pivot;
            lfPower = thrust - sideways - pivot;
            lbPower = thrust + sideways - pivot;

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

            if(gamepad2.dpad_up && relic_extension.getCurrentPosition() < 10000){
                relic_extension.setPower(RELIC_ARM_EXTENSION_POWER);
            }else if(gamepad2.dpad_down && relic_extension.getCurrentPosition() > 250){
                relic_extension.setPower(RELIC_ARM_RETRACTION_POWER);
            }else{
                relic_extension.setPower(0);
            }

            if(gamepad2.dpad_left){
                relic_claw.setPosition(RELIC_CLAW_CLOSED);
            }else if(gamepad2.dpad_right){
                relic_claw.setPosition(RELIC_CLAW_OPENED);
            }

            if(-gamepad2.right_stick_y > 0.1 && relic_arm.getPosition() <= RELIC_ARM_GRAB_POS){
                relic_arm.setPosition(relic_arm.getPosition() + 0.005);
            }else if(-gamepad2.right_stick_y < -0.1 && relic_arm.getPosition() >= 0){
                relic_arm.setPosition(relic_arm.getPosition() - 0.005);
            }

            if(-gamepad2.left_stick_y > 0.1 && relic_tilt.getPosition() <= 0.995){
                relic_tilt.setPosition(relic_tilt.getPosition() + 0.005);
            }else if(-gamepad2.left_stick_y < -0.1 && relic_tilt.getPosition() >= 0.005){
                relic_tilt.setPosition(relic_tilt.getPosition() - 0.005);
            }

            telemetry.addData("relic_extension encoders", relic_extension.getCurrentPosition());
            telemetry.update();

        }

    }
}
