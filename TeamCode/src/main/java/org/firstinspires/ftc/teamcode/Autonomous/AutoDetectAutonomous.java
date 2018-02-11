package org.firstinspires.ftc.teamcode.Autonomous;

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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.OmniDirectionalDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntake;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Jewel.TwoPointJewelArm;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.IUltrasonic;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Sarthak on 2/7/2018.
 */
@Autonomous(name = "Auto Select Autonomous", group = "Autonomous")
public class AutoDetectAutonomous extends LinearOpMode {

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
    ModernRoboticsI2cRangeSensor ultrasonic_front;
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

    final double POWER_CHANGE_GAIN = 0.003;

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

    private final double INTAKE_SPPED = 0.74;
    private final double OUTTAKE_SPEED = -0.74;

    boolean additionalDistance = false;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Init", "Starting Vuforia");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "ATXxmRr/////AAAAGdfeAuU6SEoFkpmhG616inkbnBHHQ/Ti5DMPAVykTBdmQS8ImGtoIBRRuboa+oIyuvQW1nIychXXxROjGLssEzSFF8yOYE36GqhVtRfI6lw8/HAoJpO1XgIF5Gy1vPx4KFPNInK6CJdZomYyWV8rGnb7ceLJ9Z+g0sl+VcVPKl5DAI84K+06pEZnw+Em7sThhzyzj2p4QbPhXh7fEtNGhFCqey9rcg3h9RfNebyWvJW9z7mGkaJljZy1x3lK7viLbFKyFcAaspZZi1+JzUmeuXxV0r+8hrCgFLPsvKQHlnYumazP9FEtm/FjCpRFF23Et77325/vuD2LRSPzve9ef4zqe6MivrLs9s8lUgd7Eo9W";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData("Init", "Finished Starting Vuforia");
        telemetry.update();

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

        ultrasonic_jewel = (ModernRoboticsI2cRangeSensor) hardwareMap.get("jewel_us");
        ultrasonic_back = (ModernRoboticsI2cRangeSensor) hardwareMap.get("back_us");
        ultrasonic_front = (ModernRoboticsI2cRangeSensor) hardwareMap.get("bottom_front_us");

        boolean selected = false;
        boolean aligned = false;
        String autoProgram = "";
        while(!selected&&!isStopRequested()){
            //telemetry.addData("Floor Red", floor_color.red());
            //telemetry.addData("Floor Blue", floor_color.blue());
            //telemetry.addData("Floor HSV", floor_color.getHSV()[0]);
            telemetry.addData("Auto Program Selected", autoProgram);

            if(isStopRequested()){
                selected = true;
            }
            if (gamepad1.b || gamepad2.b){
                selected = true;
            }
            if(aligned){
                led.turnOn();
            }else{
                led.turnOff();
            }

            double jewelValue = ultrasonic_jewel.cmUltrasonic();
            double backValue = ultrasonic_back.cmUltrasonic();
            double frontValue = ultrasonic_front.cmUltrasonic();

            telemetry.addData("Jewel US", jewelValue);
            telemetry.addData("Back US", backValue);
            telemetry.addData("Front US", frontValue);
            telemetry.addData("Aligned", aligned);

            if(jewelValue != 255 && backValue != 255){
                if(floor_color.red() > 35 && floor_color.getHSV()[0] < 30){
                    if(frontValue > 60 && backValue <= 40){
                        if(jewelValue == 36 && backValue == 37) {
                            autoProgram = "RedStone1";
                            aligned = true;
                        }else{
                            autoProgram = "RedStone1";
                            aligned = false;
                        }
                    }
                    if(frontValue > 60 && backValue > 90){
                        if(jewelValue == 36 && frontValue == 93){
                            aligned = true;
                            autoProgram = "RedStone2";
                        }else{
                            autoProgram = "RedStone2";
                            aligned = false;
                        }

                    }
                }else if (floor_color.getHSV()[0] > 200){
                    if(frontValue < 50 && backValue > 60){
                        if(jewelValue == 36 && frontValue == 38){
                            autoProgram = "BlueStone1";
                            aligned = true;
                        }else{
                            autoProgram = "BlueStone1";
                            aligned = false;
                        }
                    }
                    if(frontValue > 100 && (backValue > 85 && backValue < 150)){
                        if(jewelValue == 36 && backValue == 93){
                            autoProgram = "BlueStone2";
                            aligned = true;
                        }else{
                            autoProgram = "BlueStone2";
                            aligned = false;
                        }
                    }
                }
            }

            if(autoProgram.equals("RedStone2") && jewelValue == 36 && backValue > 75 && frontValue == 93){
                aligned = true;
                led.turnOn();
            }else if(autoProgram.equals("BlueStone2") && jewelValue == 36 && frontValue > 75 && backValue == 93){
                aligned = true;
                led.turnOn();
            }else if (autoProgram.equals("BlueStone2") || autoProgram.equals("RedStone2")){
                aligned = false;
            }
            telemetry.update();
        }

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
        telemetry.update();

        telemetry.addData("Init", "Finished");
        telemetry.update();
        led.setLEDPower(0.2);
        waitForStart();

        if(autoProgram.equals("RedStone1")){

            //Reset counters, timers, and activate Vuforia
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            led.turnOff();
            relicTrackables.activate();
            timer.reset();
            //Reset drive encoders
            drive.resetEncoders();

            //Intake Glyph
            intake.secureGlyph();
            intake.setLiftTargetPosition(200, 1);

            //Move jewel to position and read color
            jewel.setPanTiltPos(0.5, 0.21);
            timer.reset();
            while(timer.milliseconds() < 1000  && opModeIsActive()){
                telemetry.addData("Jewel", "Moving to Read Position");
                telemetry.addData("Timer", timer.milliseconds());
                telemetry.update();
            }
            jewel.readColor(5);
            intake.setLiftTargetPosition(700, 1);

            //Knock off jewel
            jewel.knockOffJewel("red");
            jewel.setPanTiltPos(0.5, 1);
            telemetry.addData("Jewel", "Done");

            //Read VuMark and determine drive distance and column
            telemetry.addData("VuMark", "Reading");
            telemetry.update();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", vuMark.toString());
                telemetry.update();
                vumarkSeen = vuMark.toString();

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                            /*

                             */
                }
            }
            //Determine VuMark distances
            if(vumarkSeen.equals("LEFT")){
                vuMarkDistance = 33;
            }else if (vumarkSeen.equals("RIGHT")){
                vuMarkDistance = 17;
            }else {
                vuMarkDistance = 26;
            }
            telemetry.addData("VuMark", "Finished");
            telemetry.update();

            //Drive to desired VuMark target
            drive.resetEncoders();
            while(drive.averageEncoders() < vuMarkDistance*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveNoIMU(0, 0.6, true, 0);
                telemetry.addData("Encoder Count", encoderAverage);
                telemetry.update();
            }
            drive.softResetEncoder();
            drive.setPowerZero();

            //Lower lift to dispense glyph
            lift.setTargetPosition(250);
            lift.setPower(1);

            //Pivot to face cryptobox
            while(drive.moveIMU(.7, 0.2, 0, 0, 0, 0, .008, 90, true, 500) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            powerChange = (2*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();
            while(drive.averageEncoders() < 2*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1000){
                drive.moveIMU(0.3, 0.2, powerChange, .15, 90, .008, 0.001, 90,
                        false, 1000);
                powerChange = (2*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Outtake glyph into cryptobox
            //intake.dispenseGlyph();
            if(vumarkSeen.equals("CENTER")){
                intake.dispenseGlyph();
            }else if(vumarkSeen.equals("LEFT")){
                intake.setIntakePower(-0.25, OUTTAKE_SPEED);
            }else if(vumarkSeen.equals("RIGHT")){
                intake.setIntakePower(OUTTAKE_SPEED, -0.25);
            }else{
                intake.dispenseGlyph();
            }
            timer.reset();
            while(timer.milliseconds() < 250 && opModeIsActive()){
                telemetry.addData("Intake", "Dispensing Glyph");
                telemetry.update();
            }
            //Back away from cryptobox
            powerChange = (8*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 8*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.6, 0.5, powerChange, .075, -90, 0.008, 0.001, 90,
                        false, 1000);
                powerChange = (8*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            intake.setIntakePowerZero();

            //Pivot to glyph pit
            while(drive.moveIMU(0.5, 0.3, 0, 0, 0, 0, 0.005, -90, true, 150) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Lower lift to intake glyphs
            lift.setTargetPosition(5);
            lift.setPower(1);

            //Drive to glyph pit and intake glyph
            intake.secureGlyph();
            powerChange = (20*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();

            //go into the cryptobox
            while(drive.averageEncoders()<20*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<1000){
                drive.moveIMU(.7, 0.35, powerChange, .035, -90, .02, 0.001, -90,
                        false, 1000);
                powerChange = (22*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();

            //wait for glyphs to come in robot
            while(opModeIsActive()&&timer.milliseconds()<500);
            timer.reset();

            //check if glyph in bottom arms of robot
            if (bottomGlyphColor.getDistance(DistanceUnit.CM)<6){
                additionalDistance = false;
                led.setLEDPower(1);
            }else{
                additionalDistance = true;
            }

            //Go get farther into glyph pit if glyph is not in the bottom
            if (additionalDistance) {
                while(drive.averageEncoders()<4*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<1000){
                    drive.moveIMU(.3, .25, powerChange, .075, -90, 0.008, 0.001, -90,
                            false, 1000);
                    powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                drive.setPowerZero();
                drive.softResetEncoder();
                timer.reset();

                while(drive.moveIMU(0.4, 0.3, 0, 0, 0, 0, .03, -75, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .015, -105, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .03, -90, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();

                }

            }

            //lift to glyph height 2
            lift.setTargetPosition(875);
            lift.setPower(1);
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();

            //Drive back to cryptobox
            powerChange = (10*COUNTS_PER_INCH) - drive.averageEncoders();
            if(additionalDistance){
                while(drive.averageEncoders()<10*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .04, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (10*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }else{
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .07, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }

            drive.softResetEncoder();
            led.setLEDPower(0);
            //go back faster
            powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .008, 0.001, -90,
                        false, 1000);
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();



            //Pivot to face cryptobox
            while(drive.moveIMU(0.5, 0.2, 0, 0, 0, 0, 0.005, 90, true, 500) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.softResetEncoder();
            intake.setIntakePowerZero();

            //Drive to cryptobox
            powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();
            while(drive.averageEncoders() < 12*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                drive.moveIMU(.7, 0.1, powerChange, .045, 90, 0.008, .043, 90,
                        false, 1000);
                powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.softResetEncoder();
            drive.setPowerZero();

            //Dispense glyph
            intake.dispenseGlyph();
            timer.reset();
            while(timer.milliseconds() < 250  && opModeIsActive()){
                telemetry.addData("Intake", "Dispensing Glyph");
                telemetry.update();
            }

            //Back away from cryptobox
            powerChange = (11 *COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 11*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.3, 0.1, powerChange, .03, -90, 0.008, 0.001, 90,
                        false, 1000);
                powerChange = (11*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.softResetEncoder();
            drive.setPowerZero();
            intake.setIntakePowerZero();

            //Lower lift to intake glyphs
            lift.setTargetPosition(5);
            lift.setPower(1);

            //Pivot to glyph pit
            while(drive.moveIMU(0.5, 0.3, 0, 0, 0, 0, 0.005, -90, true, 150) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();


            //Slide to new column
            if(!vumarkSeen.equals("RIGHT")){
                powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(1, 1, powerChange, .125, 180, .02, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }else{
                powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(1, 1, powerChange, .125, 0, .02, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            //Drive to glyph pit and intake glyph
            intake.secureGlyph();
            powerChange = (20*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();

            //go into the cryptobox
            while(drive.averageEncoders()<20*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<1000){
                drive.moveIMU(.7, 0.35, powerChange, .035, -90, .02, 0.001, -90,
                        false, 1000);
                powerChange = (20*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();

            //wait for glyphs to come in robot
            while(opModeIsActive()&&timer.milliseconds()<500);
            timer.reset();

            //check if glyph in bottom arms of robot
            if (bottomGlyphColor.getDistance(DistanceUnit.CM)<6){
                additionalDistance = false;
                led.setLEDPower(1);
            }else{
                additionalDistance = true;
            }

            //Go get farther into cryptobox if glyph is not in the bottom
            if (additionalDistance) {
                while(drive.averageEncoders()<4*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<1000){
                    drive.moveIMU(.3, .25, powerChange, .075, -90, 0.008, 0.001, -90,
                            false, 1000);
                    powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                drive.setPowerZero();
                drive.softResetEncoder();
                timer.reset();

                while(drive.moveIMU(0.4, 0.3, 0, 0, 0, 0, .03, -75, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .015, -105, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .03, -90, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }


            }

            //lift to glyph height 2
            lift.setTargetPosition(150);
            lift.setPower(1);
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();

            //Drive back to cryptobox

            if(additionalDistance){
                powerChange = (10*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<10*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .04, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (10*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }else{
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .07, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }

            drive.softResetEncoder();
            led.setLEDPower(0);

            //go back faster
            powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .1, .008, -90,
                        false, 1000);
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();



            //Pivot to face cryptobox
            while(drive.moveIMU(0.5, 0.2, 0, 0, 0, 0, 0.005, 90, true, 500) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.softResetEncoder();
            intake.setIntakePowerZero();

            //Drive to cryptobox
            powerChange = (13*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();
            while(drive.averageEncoders() < 13*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                drive.moveIMU(.7, 0.1, powerChange, .4375, 90, 0.008, .008, 90,
                        false, 1000);
                powerChange = (13*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.softResetEncoder();
            drive.setPowerZero();

            //Dispense glyph
            intake.dispenseGlyph();
            timer.reset();
            while(timer.milliseconds() < 250  && opModeIsActive()){
                telemetry.addData("Intake", "Dispensing Glyph");
                telemetry.update();
            }

            //Back away from cryptobox
            powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 4*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.3, 0.1, powerChange, .15, -90, 0.008, 0.001, 90,
                        false, 1000);
                powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.softResetEncoder();
            drive.setPowerZero();
            intake.setIntakePowerZero();

            //Lower lift to intake glyphs
            lift.setTargetPosition(5);
            lift.setPower(1);
            while(opModeIsActive());

        }else if (autoProgram.equals("RedStone2")){
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            led.turnOff();
            relicTrackables.activate();
            timer.reset();
            //Reset drive encoders
            drive.resetEncoders();

            //Intake Glyph and raise lift
            intake.secureGlyph();
            intake.setLiftTargetPosition(100, 1);

            //Move jewel to position and read color
            jewel.setPanTiltPos(0.5, 0.21);
            timer.reset();
            while(timer.milliseconds() < 1000&&opModeIsActive()){
                telemetry.addData("Jewel", "Moving to Read Position");
                telemetry.addData("Timer", timer.milliseconds());
                telemetry.update();
            }
            jewel.readColor(5);
            intake.setLiftTargetPosition(700, 1);
            //Knock off jewel
            jewel.knockOffJewel("red");
            jewel.setPanTiltPos(0.5, 1);
            telemetry.addData("Jewel", "Done");

            //Read VuMark and determine drive distance and column
            telemetry.addData("VuMark", "Reading");
            telemetry.update();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", vuMark.toString());
                telemetry.update();
                vumarkSeen = vuMark.toString();

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                            /*

                             */
                }
            }
            //Determine VuMark distances
            if(vumarkSeen.equals("LEFT")){
                vuMarkDistance = 20;
            }else if (vumarkSeen.equals("RIGHT")){
                vuMarkDistance = 6;
            }else {
                vuMarkDistance = 13;
            }
            telemetry.addData("VuMark", "Finished");
            telemetry.update();

            //Drive off of balancing stone
            powerChange = (16*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 16 * COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.5, 0.3, powerChange, POWER_CHANGE_GAIN, 0, 0.008, 0.001, 0,
                        false, 1000);
                powerChange = (16*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Drive to desired cryptobox column
            powerChange = (vuMarkDistance*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < vuMarkDistance*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, -90, 0.008, 0.001, 0,
                        false, 1000);
                powerChange = (vuMarkDistance*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Lower lift to deposit glyph
            lift.setTargetPosition(250);
            lift.setPower(1);
            timer.reset();


            //Drive to deposit glyph
            timer.reset();
            powerChange = (5*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 5*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                drive.moveIMU(0.3, 0.1, powerChange, POWER_CHANGE_GAIN, 0, 0.008, 0.001, 0,
                        false, 1000);
                powerChange = (5*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Deposit glyph
            intake.dispenseGlyph();
            timer.reset();
            while(timer.milliseconds() < 1500&opModeIsActive()){
                //Wait for glyph to be scored
            }

            //Back away from cryptobox
            powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 4*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.3, 0.1, powerChange, POWER_CHANGE_GAIN, 180, 0.008, 0.001, 0,
                        false, 1000);
                powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            intake.setIntakePowerZero();

            //Slide towards middle of the field
            powerChange = ((27-vuMarkDistance)*COUNTS_PER_INCH - drive.averageEncoders());
            while(drive.averageEncoders() < (27-vuMarkDistance)*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, -90, 0.008, 0.001, 0,
                        false, 1000);
                powerChange = ((27-vuMarkDistance)*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            lift.setTargetPosition(5);
            lift.setPower(1);

            //Pivot to face glyph pit
            while(drive.moveIMU(0.5, 0.2, 0, 0, 0, 0, 0.005, -135, true, 0) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Slide to pit
            powerChange = (24*COUNTS_PER_INCH - drive.averageEncoders());
            while(drive.averageEncoders() < 24*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, 165, 0.03, 0.001, -135,
                        false, 1000);
                powerChange = (24*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            intake.secureGlyph();


            //Move into pit
            powerChange = (7*COUNTS_PER_INCH - drive.averageEncoders());
            timer.reset();
            while(drive.averageEncoders() < 7*COUNTS_PER_INCH && opModeIsActive() && timer.seconds() < 2000){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, -135, .02, 0.001, -135,
                        false, 1000);
                powerChange = (7*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            if(bottomGlyphColor.getDistance(DistanceUnit.CM)<6){
                additionalDistance = false;
                led.turnOn();
            }else{
                additionalDistance = true;

            }
            if(additionalDistance){
                timer.reset();
                powerChange = (6*COUNTS_PER_INCH - drive.averageEncoders());
                while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1000){
                    drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, -135, .02, 0.001, -135,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }

                drive.setPowerZero();
                drive.softResetEncoder();
                timer.reset();

                while(drive.moveIMU(0.4, 0.3, 0, 0, 0, 0, .03, -150, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .015, -120, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .03, -135, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            if(vumarkSeen.equals("LEFT")){
                lift.setTargetPosition(900);
            }else{
                lift.setTargetPosition(150);
            }
            //lift to glyph height 2

            lift.setPower(1);
            timer.reset();

            //Drive back to cryptobox
            if(additionalDistance){
                powerChange = (7*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<12*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .04, 45, .04, 0.001, -135,
                            false, 1000);
                    powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }else{
                powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .07, 45, .04, 0.001, -135,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Slide away from
            powerChange = (24*COUNTS_PER_INCH - drive.averageEncoders());
            while(drive.averageEncoders() < 24*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, -17, 0.03, 0.001, -135,
                        false, 1000);
                powerChange = (24*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Pivot to cryptobox
            while(drive.moveIMU(0.5, 0.2, 0, 0, 0, 0, 0.005, 0, true, 500) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Slide to cryptobox
            powerChange = (6*COUNTS_PER_INCH - drive.averageEncoders());
            while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, 90, 0.03, 0.001, 0,
                        false, 1000);
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();


            //Slide to cryptobox
            timer.reset();
            powerChange = (5*COUNTS_PER_INCH - drive.averageEncoders());
            while(drive.averageEncoders() < 5*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                drive.moveIMU(0.3, 0.1, powerChange, POWER_CHANGE_GAIN, 0, 0.03, 0.001, 0,
                        false, 1000);
                powerChange = (5*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Deposit glyph
            intake.dispenseGlyph();
            timer.reset();
            while(timer.milliseconds() < 1500 && opModeIsActive()){
                //Wait for glyph to be scored
            }

            //Back away from cryptobox
            powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 4*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.3, 0.1, powerChange, POWER_CHANGE_GAIN, 180, 0.008, 0.001, 0,
                        false, 1000);
                powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            intake.setIntakePowerZero();
            lift.setTargetPosition(0);

            //End of program
            while(opModeIsActive()){
                intake.setIntakePowerZero();
                led.turnOn();
                telemetry.addData("Program", "Finished");
                telemetry.addData("VuMark Seen", vumarkSeen);
                telemetry.update();
            }

        }else if (autoProgram.equals("BlueStone1")){
            //Reset counters, timers, and activate Vuforia
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            led.turnOff();
            relicTrackables.activate();
            timer.reset();
            //Reset drive encoders
            drive.resetEncoders();

            //Intake Glyph
            intake.secureGlyph();
            intake.setLiftTargetPosition(200, 1);

            //Move jewel to position and read color
            jewel.setPanTiltPos(0.5, 0.21);
            timer.reset();
            while(timer.milliseconds() < 1000  && opModeIsActive()){
                telemetry.addData("Jewel", "Moving to Read Position");
                telemetry.addData("Timer", timer.milliseconds());
                telemetry.update();
            }
            jewel.readColor(5);
            intake.setLiftTargetPosition(700, 1);

            //Knock off jewel
            jewel.knockOffJewel("blue");
            jewel.setPanTiltPos(0.5, 1);
            telemetry.addData("Jewel", "Done");

            //Read VuMark and determine drive distance and column
            telemetry.addData("VuMark", "Reading");
            telemetry.update();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", vuMark.toString());
                telemetry.update();
                vumarkSeen = vuMark.toString();

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                            /*

                             */
                }
            }
            //Determine VuMark distances
            if(vumarkSeen.equals("LEFT")){
                vuMarkDistance = 21;
            }else if (vumarkSeen.equals("RIGHT")){
                vuMarkDistance = 37;
            }else {
                vuMarkDistance = 30;
            }
            telemetry.addData("VuMark", "Finished");
            telemetry.update();

            //Drive to desired VuMark target
            drive.resetEncoders();
            powerChange = (vuMarkDistance*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < vuMarkDistance*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.3, powerChange, .15, 180, .005, 0.001, 0,
                        false, 1000);
                powerChange = (vuMarkDistance*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            //Lower lift to dispense glyph
            lift.setTargetPosition(250);
            lift.setPower(1);

            //Pivot to face cryptobox
            while(drive.moveIMU(.7, 0.2, 0, 0, 0, 0, .008, 90, true, 500) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Drive to glyph release location
            powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();
            while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1000){
                drive.moveIMU(0.3, 0.2, powerChange, .15, 90, .008, 0.001, 90,
                        false, 1000);
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Outtake glyph into cryptobox
            intake.dispenseGlyph();
            timer.reset();
            while(timer.milliseconds() < 250 && opModeIsActive()){
                telemetry.addData("Intake", "Dispensing Glyph");
                telemetry.update();
            }
            //Back away from cryptobox
            powerChange = (8*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 8*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.6, 0.5, powerChange, .075, -90, 0.008, 0.001, 90,
                        false, 1000);
                powerChange = (8*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            intake.setIntakePowerZero();

            //Pivot to glyph pit
            while(drive.moveIMU(0.5, 0.3, 0, 0, 0, 0, 0.005, -90, true, 150) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Lower lift to intake glyphs
            lift.setTargetPosition(5);
            lift.setPower(1);

            //Drive to glyph pit and intake glyph
            intake.secureGlyph();
            powerChange = (20*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();

            //go into the cryptobox
            while(drive.averageEncoders()<20*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<1000){
                drive.moveIMU(.7, 0.35, powerChange, .035, -90, .02, 0.001, -90,
                        false, 1000);
                powerChange = (22*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();

            //wait for glyphs to come in robot
            while(opModeIsActive()&&timer.milliseconds()<500);
            timer.reset();

            //check if glyph in bottom arms of robot
            if (bottomGlyphColor.getDistance(DistanceUnit.CM)<6){
                additionalDistance = false;
                led.setLEDPower(1);
            }else{
                additionalDistance = true;
            }

            //Go get farther into glyph pit if glyph is not in the bottom
            if (additionalDistance) {
                while(drive.averageEncoders()<4*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<1000){
                    drive.moveIMU(.3, .25, powerChange, .075, -90, 0.008, 0.001, -90,
                            false, 1000);
                    powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                drive.setPowerZero();
                drive.softResetEncoder();
                timer.reset();

                while(drive.moveIMU(0.4, 0.3, 0, 0, 0, 0, .03, -75, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .015, -105, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .03, -90, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();

                }

            }

            //lift to glyph height 2
            lift.setTargetPosition(925);
            lift.setPower(1);
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();

            //Drive back to cryptobox
            powerChange = (10*COUNTS_PER_INCH) - drive.averageEncoders();
            if(additionalDistance){
                while(drive.averageEncoders()<10*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .04, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (10*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }else{
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .07, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }

            drive.softResetEncoder();
            led.setLEDPower(0);
            //go back faster
            powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .008, 0.001, -90,
                        false, 1000);
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();



            //Pivot to face cryptobox
            while(drive.moveIMU(0.5, 0.2, 0, 0, 0, 0, 0.005, 90, true, 500) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.softResetEncoder();
            intake.setIntakePowerZero();

            //Drive to cryptobox
            powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();
            while(drive.averageEncoders() < 12*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                drive.moveIMU(.7, 0.1, powerChange, .045, 90, 0.008, .043, 90,
                        false, 1000);
                powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.softResetEncoder();
            drive.setPowerZero();

            //Dispense glyph
            intake.dispenseGlyph();
            timer.reset();
            while(timer.milliseconds() < 250  && opModeIsActive()){
                telemetry.addData("Intake", "Dispensing Glyph");
                telemetry.update();
            }

            //Back away from cryptobox
            powerChange = (11 *COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 11*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.3, 0.1, powerChange, .03, -90, 0.008, 0.001, 90,
                        false, 1000);
                powerChange = (11*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.softResetEncoder();
            drive.setPowerZero();
            intake.setIntakePowerZero();

            //Lower lift to intake glyphs
            lift.setTargetPosition(5);
            lift.setPower(1);

            //Pivot to glyph pit
            while(drive.moveIMU(0.5, 0.3, 0, 0, 0, 0, 0.005, -90, true, 150) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();


            //Slide to new column
            if(!vumarkSeen.equals("RIGHT")){
                powerChange = (8*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 7*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(1, 1, powerChange, .125, 180, .02, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }else{
                powerChange = (8*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 7*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(1, 1, powerChange, .125, 0, .02, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            //Drive to glyph pit and intake glyph
            intake.secureGlyph();
            powerChange = (20*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();

            //go into the cryptobox
            while(drive.averageEncoders()<20*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<1000){
                drive.moveIMU(.7, 0.35, powerChange, .035, -90, .02, 0.001, -90,
                        false, 1000);
                powerChange = (20*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();

            //wait for glyphs to come in robot
            while(opModeIsActive()&&timer.milliseconds()<500);
            timer.reset();

            //check if glyph in bottom arms of robot
            if (bottomGlyphColor.getDistance(DistanceUnit.CM)<6){
                additionalDistance = false;
                led.setLEDPower(1);
            }else{
                additionalDistance = true;
            }

            //Go get farther into cryptobox if glyph is not in the bottom
            if (additionalDistance) {
                while(drive.averageEncoders()<4*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<1000){
                    drive.moveIMU(.3, .25, powerChange, .075, -90, 0.008, 0.001, -90,
                            false, 1000);
                    powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                drive.setPowerZero();
                drive.softResetEncoder();
                timer.reset();

                while(drive.moveIMU(0.4, 0.3, 0, 0, 0, 0, .03, -75, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .015, -105, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .03, -90, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }


            }

            //lift to glyph height 2
            lift.setTargetPosition(150);
            lift.setPower(1);
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();

            //Drive back to cryptobox

            if(additionalDistance){
                powerChange = (10*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<10*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .04, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (10*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }else{
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .07, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }

            drive.softResetEncoder();
            led.setLEDPower(0);

            //go back faster
            powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .1, .008, -90,
                        false, 1000);
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();



            //Pivot to face cryptobox
            while(drive.moveIMU(0.5, 0.2, 0, 0, 0, 0, 0.005, 90, true, 500) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.softResetEncoder();
            intake.setIntakePowerZero();

            //Drive to cryptobox
            powerChange = (14*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();
            while(drive.averageEncoders() < 14*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                drive.moveIMU(.7, 0.1, powerChange, .4375, 90, 0.008, .008, 90,
                        false, 1000);
                powerChange = (14*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.softResetEncoder();
            drive.setPowerZero();

            //Dispense glyph
            intake.dispenseGlyph();
            timer.reset();
            while(timer.milliseconds() < 250  && opModeIsActive()){
                telemetry.addData("Intake", "Dispensing Glyph");
                telemetry.update();
            }

            //Back away from cryptobox
            powerChange = (3*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 3*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.2, 0.2, powerChange, .15, -90, 0.008, 0.001, 90,
                        false, 1000);
                powerChange = (3*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.softResetEncoder();
            drive.setPowerZero();
            intake.setIntakePowerZero();

            //Lower lift to intake glyphs
            lift.setTargetPosition(5);
            lift.setPower(1);
            while(opModeIsActive());

        }else if (autoProgram.equals("BlueStone2")){
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            led.turnOff();
            relicTrackables.activate();
            timer.reset();
            //Reset drive encoders
            drive.resetEncoders();

            //Intake Glyph and raise lift
            intake.secureGlyph();
            intake.setLiftTargetPosition(100, 1);

            //Move jewel to position and read color
            jewel.setPanTiltPos(0.5, 0.21);
            timer.reset();
            while(timer.milliseconds() < 1000&&opModeIsActive()){
                telemetry.addData("Jewel", "Moving to Read Position");
                telemetry.addData("Timer", timer.milliseconds());
                telemetry.update();
            }
            jewel.readColor(5);
            intake.setLiftTargetPosition(800, 1);
            //Knock off jewel
            jewel.knockOffJewel("blue");
            jewel.setPanTiltPos(0.5, 1);
            telemetry.addData("Jewel", "Done");

            //Read VuMark and determine drive distance and column
            telemetry.addData("VuMark", "Reading");
            telemetry.update();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", vuMark.toString());
                telemetry.update();
                vumarkSeen = vuMark.toString();

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                            /*

                             */
                }
            }
            //Determine VuMark distances
            if(vumarkSeen.equals("LEFT")){
                vuMarkDistance = 6;
            }else if (vumarkSeen.equals("RIGHT")){
                vuMarkDistance = 20;
            }else {
                vuMarkDistance = 12;
            }
            telemetry.addData("VuMark", "Finished");
            telemetry.update();

            relic.setArmPosition(0.5);
            relic.setArmPosition(0);

            //Drive off of balancing stone
            powerChange = (16*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 16 * COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.5, 0.3, powerChange, POWER_CHANGE_GAIN, 180, 0.008, 0.001, 0,
                        false, 1000);
                powerChange = (16*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Drive to desired cryptobox column
            powerChange = (vuMarkDistance*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < vuMarkDistance*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, -90, 0.008, 0.001, 0,
                        false, 1000);
                powerChange = (vuMarkDistance*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Pivot to face glyph pit
            while(drive.moveIMU(0.5, 0.2, 0, 0, 0, 0, 0.005, 180, true, 0) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Lower lift to deposit glyph
            lift.setTargetPosition(250);
            lift.setPower(1);
            timer.reset();


            //Drive to deposit glyph
            timer.reset();
            powerChange = (5*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 5*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                drive.moveIMU(0.3, 0.1, powerChange, POWER_CHANGE_GAIN, 180, 0.008, 0.001, 180,
                        false, 1000);
                powerChange = (5*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Deposit glyph
            intake.dispenseGlyph();
            timer.reset();
            while(timer.milliseconds() < 1500&&opModeIsActive()){
                //Wait for glyph to be scored
            }

            //Back away from cryptobox
            powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 4*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.3, 0.1, powerChange, POWER_CHANGE_GAIN, 0, 0.008, 0.001, 180,
                        false, 1000);
                powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            intake.setIntakePowerZero();

            //Slide towards middle of the field
            powerChange = ((24-vuMarkDistance)*COUNTS_PER_INCH - drive.averageEncoders());
            while(drive.averageEncoders() < (24-vuMarkDistance)*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, -90, 0.008, 0.001, 180,
                        false, 1000);
                powerChange = ((24-vuMarkDistance)*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            lift.setTargetPosition(5);
            lift.setPower(1);

            //Pivot to face glyph pit
            while(drive.moveIMU(0.5, 0.2, 0, 0, 0, 0, 0.005, -35, true, 0) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Slide to pit
            powerChange = (24*COUNTS_PER_INCH - drive.averageEncoders());
            while(drive.averageEncoders() < 24*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, 10, 0.03, 0.001, -35,
                        false, 1000);
                powerChange = (24*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            intake.secureGlyph();


            //Move into pit
            powerChange = (9*COUNTS_PER_INCH - drive.averageEncoders());
            timer.reset();
            while(drive.averageEncoders() < 9*COUNTS_PER_INCH && opModeIsActive() && timer.seconds() < 2000){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, -35, .02, 0.001, -35,
                        false, 1000);
                powerChange = (9*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            if(bottomGlyphColor.getDistance(DistanceUnit.CM)<6){
                additionalDistance = false;
                led.turnOn();
            }else{
                additionalDistance = true;

            }
            if(additionalDistance){
                timer.reset();
                powerChange = (6*COUNTS_PER_INCH - drive.averageEncoders());
                while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1000){
                    drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, -35, .02, 0.001, -35,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }

                drive.setPowerZero();
                drive.softResetEncoder();
                timer.reset();

                while(drive.moveIMU(0.4, 0.3, 0, 0, 0, 0, .03, -50, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .015, -15, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
                while(drive.moveIMU(.4, 0.3, 0, 0, 0, 0, .03, -35, true, 0) && opModeIsActive()){
                    telemetry.addData("Move", "Pivot");
                    telemetry.update();
                }
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            if(vumarkSeen.equals("RIGHT")){
                lift.setTargetPosition(900);
            }else{
                lift.setTargetPosition(150);
            }
            //lift to glyph height 2

            lift.setPower(1);
            timer.reset();

            //Drive back to cryptobox
            if(additionalDistance){
                powerChange = (8*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<10*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .04, 145, .04, 0.001, -35,
                            false, 1000);
                    powerChange = (10*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }else{
                powerChange = (2*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1500){
                    drive.moveIMU(.4, 0.15, powerChange, .07, 145, .04, 0.001, -35,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Slide away from pit
            powerChange = (22*COUNTS_PER_INCH - drive.averageEncoders());
            while(drive.averageEncoders() < 22*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, -170, 0.005, 0.001, -35,
                        false, 1000);
                powerChange = (22*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Pivot to cryptobox
            while(drive.moveIMU(0.5, 0.2, 0, 0, 0, 0, 0.005, 180, true, 500) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Slide to cryptobox
            powerChange = (6*COUNTS_PER_INCH - drive.averageEncoders());
            while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.5, powerChange, POWER_CHANGE_GAIN, 90, 0.008, 0.001, 180,
                        false, 1000);
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();


            //Go to deposit glyph
            timer.reset();
            powerChange = (7*COUNTS_PER_INCH - drive.averageEncoders());
            while(drive.averageEncoders() < 7*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                drive.moveIMU(0.3, 0.1, powerChange, POWER_CHANGE_GAIN, 180, 0.03, 0.001, 180,
                        false, 1000);
                powerChange = (7*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Deposit glyph
            intake.dispenseGlyph();
            timer.reset();
            while(timer.milliseconds() < 1500&&opModeIsActive()){
                //Wait for glyph to be scored
            }

            //Back away from cryptobox
            powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 4*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.3, 0.1, powerChange, POWER_CHANGE_GAIN, 0, 0.008, 0.001, 180,
                        false, 1000);
                powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            intake.setIntakePowerZero();
            lift.setTargetPosition(0);

            //End of program
            while(opModeIsActive()){
                intake.setIntakePowerZero();
                led.turnOn();
                telemetry.addData("Program", "Finished");
                telemetry.addData("VuMark Seen", vumarkSeen);
                telemetry.update();
            }

        }

    }
}
