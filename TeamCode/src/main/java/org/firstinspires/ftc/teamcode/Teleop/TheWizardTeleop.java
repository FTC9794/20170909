package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    DigitalChannel glyphLimit;
    DcMotor lift;
    DcMotor rf, rb, lf, lb;

    FourArmRotatingGlyph glyph;
    IDrivetrain drive;
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

    final double ROTATION_TIME = 500;





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

        driveMotors = new ArrayList<>();
        driveMotors.add(rf);
        driveMotors.add(rb);
        driveMotors.add(lf);
        driveMotors.add(lb);

        telemetry.addData("Init", "IMU Calibrating");
        telemetry.update();
        //Initialize BOSCH IMU
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        boschIMU.initialize(parameters);
        imu = new BoschIMU(boschIMU);
        imu.setAsZero();
        telemetry.addData("Init", "IMU Instantiated");
        telemetry.update();

        drive = new OmniDirectionalDrive(driveMotors, imu);
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

            drive.move(1, 0, gamepadPlus1.getDistanceFromCenterLeft(), 1, gamepadPlus1.getAngleLeftStick(), gamepadPlus1.getDistanceFromCenterRight()*.02, 0, imu.getZAngle()+gamepadPlus1.rightStickX() * 40, false, 0);


        }

        drive.stop();

    }
}
