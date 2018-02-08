package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.DataLogger;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.OmniDirectionalDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntake;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.NavxIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Jewel.TwoPointJewelArm;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.IUltrasonic;

import java.util.ArrayList;
import java.util.Date;
import java.util.List;

/**
 * Created by Sarthak on 10/6/2017.
 */

@Autonomous(name = "Drive Test", group = "Test")
public class OmniDirectionalAutonomousDemo extends LinearOpMode {
    BNO055IMU boschIMU;
    DualWheelIntake intake;
    ClawThreePoint relic;
    IColorSensor color;
    IColorSensor floor_color;
    TwoPointJewelArm jewel;
    IUltrasonic jewel_us;
    IUltrasonic back_us;
    LED led;


    VuforiaLocalizer vuforia;

    Servo pan, tilt;
    CRServo rightWheel1, rightWheel2, leftWheel1, leftWheel2;
    Servo spin;
    DcMotor rf, rb, lf, lb;
    List<DcMotor> motors;
    DcMotor lift;
    DcMotor relic_extension;
    Servo relic_claw, relic_arm, relic_tilt;
    DigitalChannel glyphLimit;
    LynxI2cColorRangeSensor lynx, lynx_floor, bottomGlyphColor;
    IIMU imu;
    OmniDirectionalDrive drive;
    ModernRoboticsI2cRangeSensor ultrasonic_jewel;
    ModernRoboticsI2cRangeSensor ultrasonic_back;
    DcMotor leds;

    ElapsedTime timer;
    String vumarkSeen = "";
    double vuMarkDistance = 36;

    final double GRIP_OPEN1 = .5;
    final double GRIP_OPEN2 = .5;
    final double GRIP_CLOSE1 = 0;
    final double GRIP_CLOSE2 = 0;

    final double SPIN_START = 0;
    final double SPIN_ROTATED = .95;

    final double RELIC_CLAW_CLOSED = 1;
    final double RELIC_CLAW_OPENED = 0;

    final double RELIC_TILT_ORIGIN = 1;

    final double RELIC_ARM_ORIGIN = 0;

    final double COUNTS_PER_INCH = 45;
    final double ENCODER_OFFSET = 30;

    double imuAngle, encoderAverage, powerChange = 0;

    final int RED_LINE_COLOR_VALUE = 50;

    final double LIFT_POWER_UP = 1;
    final double LIFT_POWER_DOWN = -1;

    boolean additionalDistance = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialize Right Motors
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");

        //initialize left motors
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift = hardwareMap.dcMotor.get("glyph_lift");

        lynx = (LynxI2cColorRangeSensor) hardwareMap.get("jewel_color");
        lynx_floor = (LynxI2cColorRangeSensor) hardwareMap.get("floor_color");
        bottomGlyphColor = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor1");
        pan = hardwareMap.servo.get("jewel_pan");
        tilt = hardwareMap.servo.get("jewel_tilt");

        rightWheel1 = hardwareMap.crservo.get("right_glyph1");
        leftWheel1 = hardwareMap.crservo.get("left_glyph1");
        //leftWheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel2 = hardwareMap.crservo.get("right_glyph2");
        leftWheel2 = hardwareMap.crservo.get("left_glyph2");

        spin = hardwareMap.servo.get("spin_grip");

        relic_extension = hardwareMap.dcMotor.get("relic_extension");
        relic_extension.setDirection(DcMotorSimple.Direction.REVERSE);
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relic_extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relic_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        relic_arm = hardwareMap.servo.get("relic_arm");
        relic_claw = hardwareMap.servo.get("relic_claw");
        relic_tilt = hardwareMap.servo.get("relic_tilt");


        color = new LynxColorRangeSensor(lynx);
        floor_color = new LynxColorRangeSensor(lynx_floor);
        jewel = new TwoPointJewelArm(pan, tilt, color, telemetry);
        relic = new ClawThreePoint(relic_extension, relic_arm, relic_tilt, relic_claw);
        relic.setTiltPosition(1);
        relic_arm.setPosition(0);
        telemetry.addData("Init", "Jewel, Relic Hardware Initialized");
        telemetry.update();
        jewel.setPanTiltPos(0.5, 1);
        telemetry.addData("Init", "Jewel Servos Set");
        telemetry.update();

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glyphLimit = hardwareMap.digitalChannel.get("glyph_limit");
        intake = new DualWheelIntake(rightWheel1, rightWheel2, leftWheel1, leftWheel2, spin, lift, glyphLimit, telemetry);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Init", "Initialized Intake System");
        telemetry.update();

        //create array list of motors
        motors = new ArrayList<>();
        motors.add(rf);
        motors.add(rb);
        motors.add(lf);
        motors.add(lb);


        leds = hardwareMap.dcMotor.get("leds");
        led = new LED(leds);

        //set motor modes and zero power behavior
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        telemetry.addData("Init", "Set Drivetrain mode");
        telemetry.update();

        //Calibrate IMU
        telemetry.addData("Init", "IMU Calibrating");
        telemetry.update();
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Init", "IMU Instantiated");
        telemetry.update();

        //initialize drivetrain
        drive = new OmniDirectionalDrive(motors, imu, telemetry);
        drive.resetEncoders();
        telemetry.addData("Init", "Drivetrain and IMU Initialized");
        telemetry.update();

        //Finish init
        timer = new ElapsedTime();
        telemetry.addData("Init", "Timer Initialized");
        telemetry.addData("Init", "Completed");
        telemetry.update();
        led.setLEDPower(0.5);

        Date day = new Date();
        DataLogger data = new DataLogger(day.toString() + " OmniDriveTest");
        data.addField("Color Sensor Red Value");
        data.addField("Color Sensor Blue Value");
        data.addField("Color Sensor Hue");
        data.newLine();

        waitForStart();
        drive.softResetEncoder();

        timer.reset();
        //go into the cryptobox
        powerChange = (20*COUNTS_PER_INCH) - drive.averageEncoders();
        while(drive.averageEncoders()<20*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<3000){
            drive.moveIMU(.7, 0.35, powerChange, .035, -90, .02, 0.001, -90,
                    false, 1000);
            powerChange = (20*COUNTS_PER_INCH) - drive.averageEncoders();
            data.addField(floor_color.red());
            data.addField(floor_color.blue());
            data.addField(floor_color.getHSV()[0]);
            data.newLine();
        }
        drive.setPowerZero();
        timer.reset();

        while(opModeIsActive()){
            telemetry.addData("Distance Traveled", drive.averageEncoders()/COUNTS_PER_INCH);
            telemetry.update();

        }

    }
}
