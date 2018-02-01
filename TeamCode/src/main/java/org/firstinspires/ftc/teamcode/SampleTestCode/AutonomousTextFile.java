package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.apache.commons.jexl3.JexlBuilder;
import org.apache.commons.jexl3.JexlContext;
import org.apache.commons.jexl3.JexlEngine;
import org.apache.commons.jexl3.JexlExpression;
import org.apache.commons.jexl3.MapContext;
import org.apache.commons.jexl3.internal.introspection.EnumerationIterator;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AcceleratedDcMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.OmniDirectionalDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.FourArmRotatingGlyph;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.NavxIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Jewel.TwoPointJewelArm;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.IUltrasonic;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.MRRangeSensor;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by Sarthak on 10/22/2017.
 */
@Autonomous(name = "Autonomous Text File", group = "Test")
@Disabled
public class AutonomousTextFile extends LinearOpMode {

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
    LynxI2cColorRangeSensor lynx, lynx_floor;
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


    JexlEngine jexl = new JexlBuilder().create();
    String autoFileName = "RedStone2.txt";
    File autoFile = AppUtil.getInstance().getSettingsFile(autoFileName);
    String fileText = "";
    String[] inputs;
    String[][] lookupTable;
    int lookupCount = 0;
    final int STATE_NAME = 0;
    final int STATE_CONDITION = 1;
    final int STATE_ACTION = 2;
    final double COUNTS_PER_INCH = 45;
    final double ENCODER_OFFSET = 30;

    double imuAngle, encoderAverage;

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

        telemetry.addData("Init", "reading file");
        telemetry.update();

        boolean autoSelected = false;

        fileText = ReadWriteFile.readFile(autoFile);
        inputs = fileText.split("~");
        lookupTable = new String[100][100];
        for(int i = 0; i<inputs.length; i++){
            String currentLine = inputs[i];
            String[] currentParams = currentLine.split("/");

            for(int j = 0; j< currentParams.length; j++){
                lookupTable[i][j] = currentParams[j];
            }

        }

        //parameters = inputs[1].split("/");

        telemetry.addData("Init", "File read, data stored");
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
        pan = hardwareMap.servo.get("jewel_pan");
        tilt = hardwareMap.servo.get("jewel_tilt");

        rightWheel1 = hardwareMap.crservo.get("right_glyph1");
        leftWheel1 = hardwareMap.crservo.get("left_glyph1");
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

        //set motor modes and zero power behavior
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        int aligned = 0;
        ultrasonic_jewel = (ModernRoboticsI2cRangeSensor) hardwareMap.get("jewel_us");
        jewel_us = new MRRangeSensor(ultrasonic_jewel);
        ultrasonic_back = (ModernRoboticsI2cRangeSensor) hardwareMap.get("back_us");
        back_us = new MRRangeSensor(ultrasonic_back);
        leds = hardwareMap.dcMotor.get("leds");
        led = new LED(leds);
        while(aligned == 0){
            telemetry.addData("Jewel Ultrasonic", ultrasonic_jewel.cmUltrasonic());
            telemetry.addData("Back Ultrasnoic", ultrasonic_back.cmUltrasonic());
            if(ultrasonic_back.cmUltrasonic() == 37 && ultrasonic_jewel.cmUltrasonic() == 36){
                telemetry.addData("Alinged", "True");
                led.turnOn();
            }else{
                telemetry.addData("Aligned", "False");
                led.turnOff();
            }
            telemetry.update();
            if(isStopRequested()){
                aligned = 1;
            }else if(gamepad1.a){
                aligned = 1;
            }
            telemetry.update();
        }
        telemetry.update();

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

        timer = new ElapsedTime();
        telemetry.addData("Init", "Timer Initialized");
        telemetry.addData("Init", "Completed");
        telemetry.update();
        waitForStart();
        led.turnOff();
        relicTrackables.activate();
        timer.reset();
        while(opModeIsActive()){
            telemetry.addData("Autonomous", "In Loop, Running");
            imuAngle = imu.getZAngle();
            encoderAverage = drive.averageEncoders();

            switch (lookupTable[lookupCount][STATE_NAME].trim()){
                case "start":
                    timer.reset();
                    lookupCount++;
                    telemetry.addData("State", "Start");
                    telemetry.addData("Lookup count", lookupCount);
                    drive.resetEncoders();
                    break;
                case "wait":
                    int delay = Integer.parseInt(lookupTable[lookupCount][STATE_CONDITION]);
                    String jexlExpTime = lookupTable[lookupCount][STATE_ACTION];

                    //Create Jexl Expression and Set Context
                    JexlExpression eTime = jexl.createExpression(jexlExpTime);
                    JexlContext contextTime = new MapContext();
                    contextTime.set("timer", timer);

                    //Evaluate Expression
                    Object evaluatedExpressionObjTime = eTime.evaluate(contextTime);
                    Double elapsedTime = null;
                    if(evaluatedExpressionObjTime instanceof Double){
                        elapsedTime = (Double) evaluatedExpressionObjTime;
                    }

                    //Check Condition
                    if(elapsedTime < delay){
                        telemetry.addData("Elapsed Time", elapsedTime);
                        telemetry.addData("State", "Delay");
                    }else{
                        timer.reset();
                        lookupCount++;
                        drive.resetEncoders();
                    }
                    break;

                case "intake":
                    String jexlExpIntake = lookupTable[lookupCount][STATE_ACTION];
                    telemetry.addData("Intake", "Got Expression");
                    telemetry.update();
                    JexlExpression eIntake = jexl.createExpression(jexlExpIntake);
                    JexlContext contextIntake = new MapContext();
                    telemetry.addData("Intake", "Made Jexl Expression");
                    telemetry.update();
                    contextIntake.set("intake", intake);
                    Object evaluatedExpressionObjIntake = eIntake.evaluate(contextIntake);
                    telemetry.addData("Intake", "Evaluated Context");
                    telemetry.update();
                    Object intakeReult = evaluatedExpressionObjIntake;
                    telemetry.addData("Intake", "Assigned Result to Var");
                    telemetry.update();
                    timer.reset();
                    lookupCount++;
                    drive.resetEncoders();
                    break;

                case "jewel":
                    String jexlExpJewel = lookupTable[lookupCount][STATE_ACTION];
                    telemetry.addData("Jewel", "Got Expression");
                    telemetry.update();
                    //Create JEXL Expression
                    JexlExpression eJewel = jexl.createExpression(jexlExpJewel);
                    JexlContext contextJewel = new MapContext();
                    telemetry.addData("Jewel", "Made JEXL Expression");
                    telemetry.update();
                    if(jexlExpJewel.indexOf("reading") != -1){
                        int numReadings = Integer.parseInt(lookupTable[lookupCount][STATE_CONDITION]);
                        telemetry.addData("Jewel", "Parsed Readings");
                        telemetry.update();
                        contextJewel.set("jewel", jewel);
                        contextJewel.set("reading", numReadings);
                        telemetry.addData("Jewel", "Context Set");
                        telemetry.update();
                    }else if(jexlExpJewel.indexOf("alliance") != -1){
                        String allianceBall = lookupTable[lookupCount][STATE_CONDITION];
                        contextJewel.set("jewel", jewel);
                        contextJewel.set("alliance", allianceBall);
                    }else if (jexlExpJewel.indexOf("pan,tilt") != -1){
                        String[] servoPositions = lookupTable[lookupCount][STATE_CONDITION].split("-");
                        double panServoPos = Double.parseDouble(servoPositions[0]);
                        double tiltServoPos = Double.parseDouble(servoPositions[1]);
                        contextJewel.set("jewel", jewel);
                        contextJewel.set("pan", panServoPos);
                        contextJewel.set("tilt", tiltServoPos);
                    }
                    Object evaluatedExpressionObjJewel = eJewel.evaluate(contextJewel);
                    telemetry.addData("Jewel", "Evaluated Contex");
                    telemetry.update();
                    Object jewelResult = evaluatedExpressionObjJewel;
                    telemetry.addData("Jewel", "Assigned Evaluated Context");
                    telemetry.addData("jewel result", jewelResult);
                    telemetry.update();

                    timer.reset();
                    lookupCount++;
                    drive.resetEncoders();
                    break;

                case "vumark":
                    telemetry.addData("state", "vumark");
                    telemetry.update();
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                        telemetry.addData("VuMark", vuMark.toString());
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
                    if(vumarkSeen.equals("LEFT")){
                        if(lookupTable[lookupCount][STATE_CONDITION].equals("0")){
                            vuMarkDistance = 30;
                        }else{
                            vuMarkDistance = 16;
                        }
                        timer.reset();
                        drive.resetEncoders();
                        lookupCount++;
                    }else if (vumarkSeen.equals("RIGHT")){
                        if(lookupTable[lookupCount][STATE_CONDITION].equals("0")){
                            vuMarkDistance = 15;
                        }else{
                            vuMarkDistance = 5;
                        }
                        timer.reset();
                        drive.resetEncoders();
                        lookupCount++;
                    }else {
                        if(lookupTable[lookupCount][STATE_CONDITION].equals("0")){
                            vuMarkDistance = 22;
                        }else{
                            vuMarkDistance = 10;
                        }
                        timer.reset();
                        drive.resetEncoders();
                        lookupCount++;
                    }
                    break;

                case "move":
                    //Get condition
                    String[] slideCondition = (lookupTable[lookupCount][STATE_CONDITION].split("-"));
                    int slideCaseNum = Integer.parseInt(slideCondition[0]); //Type of slide
                    int condition = Integer.parseInt(slideCondition[1]);    //Condition to check
                    if(lookupTable[lookupCount][STATE_ACTION].indexOf("NoIMU") != -1){
                        //Get move parameters
                        String abbreviatedAction = lookupTable[lookupCount][STATE_ACTION]; //Separate parameters from rest of string
                        abbreviatedAction = abbreviatedAction.substring(16, abbreviatedAction.length()-1);
                        String[] moveParameters = abbreviatedAction.split(",");
                        double maxPower = Double.parseDouble(moveParameters[1]);        //Get Max Power
                        int moveAngle = Integer.parseInt(moveParameters[0]);            //Get Move Angle
                        String moveNoIMUString = "";
                        switch (slideCaseNum){
                            case 1: //Encoders
                                if(drive.averageEncoders() < condition*COUNTS_PER_INCH){
                                    moveNoIMUString = lookupTable[lookupCount][STATE_ACTION];
                                    JexlExpression moveNoIMUExp = jexl.createExpression(moveNoIMUString);
                                    JexlContext moveNoIMUConext = new MapContext();
                                    moveNoIMUConext.set("true", true);
                                    moveNoIMUConext.set("drive", drive);
                                    Object moveNoIMUResult = moveNoIMUExp.evaluate(moveNoIMUConext);

                                    telemetry.addData("Slide Encoders", condition*COUNTS_PER_INCH-encoderAverage + " inches left");
                                    telemetry.addData("Encoder Count", encoderAverage);
                                    telemetry.addData("Condition", condition);
                                }else{
                                    telemetry.addData("Slide", "Finished");
                                    timer.reset();
                                    drive.setPowerZero();
                                    drive.resetEncoders();
                                    lookupCount++;
                                    break;
                                }
                                break;

                            case 2: //Move for time
                                if(timer.milliseconds() < condition){
                                    moveNoIMUString = lookupTable[lookupCount][STATE_ACTION];
                                    JexlExpression moveNoIMUExp = jexl.createExpression(moveNoIMUString);
                                    JexlContext moveNoIMUConext = new MapContext();
                                    moveNoIMUConext.set("true", true);
                                    moveNoIMUConext.set("drive", drive);
                                    Object moveNoIMUResult = moveNoIMUExp.evaluate(moveNoIMUConext);
                                }else{
                                    telemetry.addData("Slide", "Finished");
                                    timer.reset();
                                    drive.setPowerZero();
                                    drive.resetEncoders();
                                    lookupCount++;
                                    break;
                                }
                                break;

                            case 3: //VuMark
                                if(drive.averageEncoders() < vuMarkDistance*COUNTS_PER_INCH){
                                    moveNoIMUString = lookupTable[lookupCount][STATE_ACTION];
                                    JexlExpression moveNoIMUExp = jexl.createExpression(moveNoIMUString);
                                    JexlContext moveNoIMUConext = new MapContext();
                                    moveNoIMUConext.set("true", true);
                                    moveNoIMUConext.set("drive", drive);
                                    Object moveNoIMUResult = moveNoIMUExp.evaluate(moveNoIMUConext);telemetry.addData("Slide Encoders", condition*COUNTS_PER_INCH-encoderAverage + " inches left");
                                    telemetry.addData("Encoder Count", encoderAverage);
                                    telemetry.addData("Condition", condition);
                                }else{
                                    telemetry.addData("Slide", "Finished");
                                    timer.reset();
                                    drive.setPowerZero();
                                    drive.resetEncoders();
                                    lookupCount++;
                                    break;
                                }
                                break;

                        }

                    }else{
                        //Get move parameters
                        String abbreviatedAction = lookupTable[lookupCount][STATE_ACTION]; //Separate parameters from rest of string
                        abbreviatedAction = abbreviatedAction.substring(14, abbreviatedAction.length()-1);
                        String[] moveParameters = abbreviatedAction.split(",");
                        double maxPower = Double.parseDouble(moveParameters[0]);        //Get Maz Power
                        double minPower = Double.parseDouble(moveParameters[1]);        //Get Min Power
                        int moveAngle = Integer.parseInt(moveParameters[2]);            //Get Move Angle
                        int orientation = Integer.parseInt(moveParameters[3]);          //Get Orientation
                        double timeAfterAngle = Double.parseDouble(moveParameters[4]);  //Get time after angle
                        double powerChange;                                             //Will be calculated within case

                        switch (slideCaseNum){
                            case 1: //Encoders
                                powerChange = (condition*COUNTS_PER_INCH) - drive.averageEncoders();
                                if(drive.moveIMU(maxPower, minPower, powerChange, 0.003, moveAngle, .035, .001, orientation,
                                        condition*COUNTS_PER_INCH - drive.averageEncoders() < ENCODER_OFFSET && condition*COUNTS_PER_INCH - drive.averageEncoders() > -ENCODER_OFFSET, timeAfterAngle)){
                                    telemetry.addData("Max Power", maxPower);
                                    telemetry.addData("Min Power", minPower);
                                    telemetry.addData("power change", powerChange);
                                    telemetry.addData("move angle", moveAngle);
                                    telemetry.addData("orientation", orientation);
                                    telemetry.addData("Slide Encoders", condition*COUNTS_PER_INCH-encoderAverage + " inches left");
                                    telemetry.addData("Encoder Count", encoderAverage);
                                    telemetry.addData("Condition", condition);
                                }else{
                                    telemetry.addData("Slide", "Finished");
                                    timer.reset();
                                    drive.setPowerZero();
                                    drive.resetEncoders();
                                    lookupCount++;
                                    break;
                                }
                                break;

                            case 2: //pivot
                                if(drive.moveIMU(maxPower, minPower, 0, 0, 0, 0, 0.005, orientation, true, 1000)){
                                    telemetry.addData("Move", "Pivot");
                                }else{
                                    telemetry.addData("Pivot", "Finished");
                                    timer.reset();
                                    drive.setPowerZero();
                                    drive.resetEncoders();
                                    lookupCount++;
                                    break;
                                }
                                break;

                            case 3: //Move for time
                                powerChange = condition - timer.seconds();
                                if(drive.moveIMU(maxPower, minPower, powerChange, 0.003, moveAngle, .025, .0001, orientation,
                                        timer.seconds() > condition, timeAfterAngle)){
                                }else{
                                    telemetry.addData("Slide", "Finished");
                                    timer.reset();
                                    drive.setPowerZero();
                                    drive.resetEncoders();
                                    lookupCount++;
                                    break;
                                }
                                break;

                            case 4: //VuMark
                                powerChange = (vuMarkDistance * COUNTS_PER_INCH) - drive.averageEncoders();
                                if(drive.moveIMU(maxPower, minPower, powerChange, 0.003, moveAngle, .035, .001, orientation,
                                        vuMarkDistance*COUNTS_PER_INCH - drive.averageEncoders() < ENCODER_OFFSET && vuMarkDistance*COUNTS_PER_INCH - drive.averageEncoders() > -ENCODER_OFFSET, timeAfterAngle)){
                                    telemetry.addData("Max Power", maxPower);
                                    telemetry.addData("Min Power", minPower);
                                    telemetry.addData("power change", powerChange);
                                    telemetry.addData("move angle", moveAngle);
                                    telemetry.addData("orientation", orientation);
                                    telemetry.addData("Slide Encoders", condition*COUNTS_PER_INCH-encoderAverage + " inches left");
                                    telemetry.addData("Encoder Count", encoderAverage);
                                    telemetry.addData("Condition", condition);
                                }else{
                                    telemetry.addData("Slide", "Finished");
                                    timer.reset();
                                    drive.setPowerZero();
                                    drive.resetEncoders();
                                    lookupCount++;
                                    break;
                                }
                                break;

                        }

                    }

                    break;

                case "stop":
                    telemetry.addData("State", "Stop");
                    telemetry.addData("IMU Angle", imuAngle);
                    telemetry.addData("Encoder Average", encoderAverage);
                    telemetry.addData("VuMark Seen", vumarkSeen);
                    break;
            }
            telemetry.update();
        }
        relicTrackables.deactivate();
        jexl.clearCache();
        drive.stop();
    }
}
