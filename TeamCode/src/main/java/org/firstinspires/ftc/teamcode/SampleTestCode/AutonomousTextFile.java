package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.FourArmRotatingGlyph;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.NavxIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Jewel.TwoPointJewelArm;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by Sarthak on 10/22/2017.
 */
@Autonomous(name = "Autonomous Text File", group = "Test")
public class AutonomousTextFile extends LinearOpMode {

    DcMotor rf, rb, lf, lb;
    DcMotor lift, relic_extension;
    BNO055IMU boschIMU;
    LynxI2cColorRangeSensor lynx;
    FourArmRotatingGlyph glyph;
    ClawThreePoint relic;

    VuforiaLocalizer vuforia;

    Servo pan, tilt;
    Servo right1, right2, left1, left2, spin;
    Servo relic_claw, relic_arm, relic_tilt;
    IIMU imu;
    IColorSensor colorSensor;
    OmniDirectionalDrive drive;
    TwoPointJewelArm jewel;
    List<DcMotor> motors;
    ElapsedTime timer;
    String vumarkSeen = "";

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
    String autoFileName = "test.txt";
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
        lift = hardwareMap.dcMotor.get("glyph_lift");

        lynx = (LynxI2cColorRangeSensor) hardwareMap.get("jewel_color");
        pan = hardwareMap.servo.get("jewel_pan");
        tilt = hardwareMap.servo.get("jewel_tilt");

        right1 = hardwareMap.servo.get("right_glyph1");
        right2 = hardwareMap.servo.get("right_glyph2");
        left1 = hardwareMap.servo.get("left_glyph1");
        left2 = hardwareMap.servo.get("left_glyph2");
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


        colorSensor = new LynxColorRangeSensor(lynx);
        jewel = new TwoPointJewelArm(pan, tilt, colorSensor, telemetry);
        telemetry.addData("Init", "Jewel Hardware Initialized");
        telemetry.update();
        jewel.setPanTiltPos(0.5, 1);
        telemetry.addData("Init", "Jewel Servos Set");
        telemetry.update();

        right2.setDirection(Servo.Direction.REVERSE);
        left1.setDirection(Servo.Direction.REVERSE);

        right1.setPosition(GRIP_OPEN1);
        right2.setPosition(GRIP_OPEN2);
        left1.setPosition(GRIP_OPEN1);
        left2.setPosition(GRIP_OPEN2);

        spin.setPosition(SPIN_START);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        glyph = new FourArmRotatingGlyph(right1, right2, left1, left2, spin, lift);
        telemetry.addData("Init", "Initialized Glyph System");
        telemetry.update();

        relic_claw.setPosition(RELIC_CLAW_CLOSED);
        relic_tilt.setPosition(RELIC_TILT_ORIGIN);
        relic_arm.setPosition(RELIC_ARM_ORIGIN);
        relic = new ClawThreePoint(relic_extension, relic_arm, relic_tilt, relic_claw, telemetry);
        telemetry.addData("Init", "Initialized Relic System");
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

        telemetry.addData("Init", "IMU Calibrating");
        telemetry.update();
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
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
        relicTrackables.activate();
        timer.reset();
        while(opModeIsActive()){
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

                    break;

                case "move":
                    //Get condition
                    String[] slideCondition = (lookupTable[lookupCount][STATE_CONDITION].split("-"));
                    int slideCaseNum = Integer.parseInt(slideCondition[0]); //Type of slide
                    int condition = Integer.parseInt(slideCondition[1]);    //Condition to check
                    //Get move parameters
                    String abbreviatedAction = lookupTable[lookupCount][STATE_ACTION]; //Separate parameters from rest of string
                    abbreviatedAction = abbreviatedAction.substring(11, abbreviatedAction.length()-1);
                    String[] moveParameters = abbreviatedAction.split(",");
                    double maxPower = Double.parseDouble(moveParameters[0]);        //Get Maz Power
                    double minPower = Double.parseDouble(moveParameters[1]);        //Get Min Power
                    int moveAngle = Integer.parseInt(moveParameters[2]);            //Get Move Angle
                    int orientation = Integer.parseInt(moveParameters[3]);          //Get Orientation
                    double timeAfterAngle = Double.parseDouble(moveParameters[4]);  //Get time after angle
                    double powerChange;                                             //Will be calculated within case

                    switch (slideCaseNum){
                        case 1: //Encoders
                            powerChange = (condition * COUNTS_PER_INCH) - drive.averageEncoders();
                            if(drive.move(maxPower, minPower, powerChange, 0.003, moveAngle, .035, .001, orientation,
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
                            if(drive.move(maxPower, minPower, 0, 0, 0, 0, 0.001, orientation, true, 1000)){
                                telemetry.addData("Move", "Pivot");
                            }else{
                                telemetry.addData("Pivot", "Finished");
                                timer.reset();
                                drive.setPowerZero();
                                drive.softResetEncoder();
                                lookupCount++;
                                break;
                            }
                            break;

                        case 3: //Move for time
                            powerChange = condition - timer.seconds();
                            if(drive.move(maxPower, minPower, powerChange, 0.003, moveAngle, .025, .0001, orientation,
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
                    }

                    break;

                case "stop":
                    telemetry.addData("State", "Stop");
                    telemetry.addData("IMU Angle", imuAngle);
                    telemetry.addData("Encoder Average", encoderAverage);
                    break;
            }
            telemetry.update();
        }
    }
}
