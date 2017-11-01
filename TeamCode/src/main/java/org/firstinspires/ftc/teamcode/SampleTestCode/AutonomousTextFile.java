package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.kauailabs.navx.ftc.AHRS;
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
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AcceleratedDcMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.OmniDirectionalDrive;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.NavxIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Jewel.TwoPointJewelArm;

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
    AHRS navx;
    LynxI2cColorRangeSensor lynx;

    Servo pan, tilt;

    IIMU imu;
    IColorSensor color;
    OmniDirectionalDrive drive;
    TwoPointJewelArm jewel;
    List<DcMotor> motors;
    ElapsedTime timer;

    JexlEngine jexl = new JexlBuilder().create();
    String autoFileName = "test.txt";
    File autoFile = AppUtil.getInstance().getSettingsFile(autoFileName);
    String fileText = "";
    String[] inputs, parameters;
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

        parameters = inputs[1].split("/");

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

        //initialize IMU
        navx  = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("Device Interface Module 1"),
                0,
                AHRS.DeviceDataType.kProcessedData);
        imu = new NavxIMU(navx);
        imu.calibrate();
        imu.setAsZero();

        //initialize drivetrain
        drive = new OmniDirectionalDrive(motors, imu, telemetry);
        drive.resetEncoders();

        telemetry.addData("Init", "IMU and Drivetrain Instantiated");
        telemetry.update();

        lynx = (LynxI2cColorRangeSensor) hardwareMap.get("color");
        //pan = hardwareMap.servo.get("servoP");
        tilt = hardwareMap.servo.get("servoT");
        color = new LynxColorRangeSensor(lynx);
        jewel = new TwoPointJewelArm(pan, tilt, color, telemetry);

        timer = new ElapsedTime();
        telemetry.addData("Init", "Timer Instantiated");
        telemetry.addData("Drive abbr", (lookupTable[1][STATE_ACTION]).substring(11, (lookupTable[1][STATE_ACTION]).length()-2));
        telemetry.update();
        waitForStart();
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
                    //Create JEXL Expression
                    JexlExpression eJewel = jexl.createExpression(jexlExpJewel);
                    JexlContext contextJewel = new MapContext();
                    if(jexlExpJewel.indexOf("reading") != -1){
                        int numReadings = Integer.parseInt(lookupTable[lookupCount][STATE_CONDITION]);
                        contextJewel.set("jewel", jewel);
                        contextJewel.set("reading", numReadings);
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
                    Object jewelResult = evaluatedExpressionObjJewel;

                    timer.reset();
                    lookupCount++;
                    drive.resetEncoders();
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
