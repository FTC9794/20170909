package org.firstinspires.ftc.teamcode.SampleTestCode;

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
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntakeThread;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Jewel.TwoPointJewelArm;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.Enums.Alliance.RED;
import static org.firstinspires.ftc.teamcode.Enums.Alliance.UNKNOWN;

/**
 * Created by Sarthak on 3/27/2018.
 */
@Autonomous(name = "Far Stone Flip Algorithm Autonomous", group = "Autonomous")
public class FarStoneFlipAutonomous extends LinearOpMode {

    //Create threads for intake mechanism
    DualWheelIntakeThread bottomIntake, topIntake;
    Thread bottomIntakeThread, topIntakeThread;
    //create mechanism interface variables
    ClawThreePoint relic;
    IColorSensor jewelColor;
    IColorSensor floor_color;
    TwoPointJewelArm jewel;
    LED led;
    BNO055IMU boschIMU;
    android.hardware.Camera cam;
    IIMU imu;
    MecanumDriveTrain drive;

    //create vuforia variables
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    String vumarkSeen = "";

    //create servo variables
    Servo pan, tilt;
    CRServo rightWheel1, rightWheel2, leftWheel1, leftWheel2;
    Servo spin;
    Servo relic_claw, relic_arm, relic_tilt;
    //Create motor variables
    DcMotor rf, rb, lf, lb;
    List<DcMotor> motors;
    DcMotor lift;
    DcMotor relic_extension;
    DcMotor leds;

    //Sensor variables
    DigitalChannel glyphLimit;
    LynxI2cColorRangeSensor jewel_color, bottom_color, bottomGlyphColor, topGlyphColor;
    ModernRoboticsI2cRangeSensor ultrasonic_jewel;
    ModernRoboticsI2cRangeSensor ultrasonic_back;
    ModernRoboticsI2cRangeSensor ultrasonic_front, ultrasonic_front_top;

    //create timers
    ElapsedTime timer;
    ElapsedTime gameTime;

    //define constants for drive movement parameters
    final double DEFAULT_MAX_POWER = .75;
    final double DEFAULT_MIN_POWER = .25;
    final double DEFAULT_ERROR_DISTANCE = 10;
    final double[] DEFAULT_PID = {.05};
    final double DEFAULT_MIN_POWER_PIVOT = .15;

    //define constants for servo positions
    final double SPIN_START = 0.825;
    final double SPIN_ROTATED = 0;

    final double RELIC_CLAW_CLOSED = 1;
    final double RELIC_CLAW_OPENED = 0;

    final double RELIC_TILT_ORIGIN = 1;

    final double RELIC_ARM_ORIGIN = 0;

    //define constants for distance movements using encoders
    final double COUNTS_PER_INCH = 45;
    final double CENTER_STONE_1_DIST = 31*COUNTS_PER_INCH;
    final double COLUMN_OFFSET = 7.5*COUNTS_PER_INCH;
    final double BLUE_ALLIANCE_OFFSET = 1.5*COUNTS_PER_INCH;

    //define variables to hold information regarding autonomous selection
    String autoProgram = "";
    Alliance alliance = UNKNOWN;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();

        //Initialize all objects before selecting autonomous
        floor_color = new LynxColorRangeSensor(bottom_color);
        led = new LED(leds);
        jewelColor = new LynxColorRangeSensor(jewel_color);
        jewel = new TwoPointJewelArm(pan, tilt, jewelColor, telemetry);
        bottomIntake = new DualWheelIntakeThread(rightWheel1, leftWheel1, bottomGlyphColor, telemetry);
        topIntake = new DualWheelIntakeThread(rightWheel2, leftWheel2, topGlyphColor, telemetry);
        bottomIntakeThread = new Thread(bottomIntake);
        topIntakeThread = new Thread(topIntake);
        relic = new ClawThreePoint(relic_extension, relic_arm, relic_tilt, relic_claw);

        setMotorBehaviors();    //set the motors to their correct modes
        initIMU();              //Initialize IMU after robot is set

        drive = new MecanumDriveTrain(motors, imu, telemetry);  //initialize drivetrain after IMU is reset

        //Init timers
        timer = new ElapsedTime();
        gameTime = new ElapsedTime();

        //Finish init
        telemetry.addData("Init", "Finished");
        telemetry.addData("Auto Program Selected", autoProgram);
        telemetry.addData("Alliance", alliance);
        telemetry.update();
        led.setLEDPower(0.2);

        //Wait for program to start
        waitForStart();

        drive.resetEncoders();

        //Start intake
        bottomIntakeThread.start();
        topIntakeThread.start();

        bottomIntake.secureGlyph();
        topIntake.secureGlyph();

        timer.reset();
        while(timer.milliseconds() < 2000 && opModeIsActive());

        drive.resetEncoders();

        int glyphCase = 0;
        //CASE 0: THE ROBOT HAS 2 GLYPH
        if(bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6 && topGlyphColor.getDistance(DistanceUnit.CM) <= 6){
            //Do nothing
            led.turnOn();
            glyphCase = 0;
        }
        //CASE 1: THE ROBOT HAS 1 GLYPH IN THE BOTTOM AND NO GLYPH IN THE TOP
        else if(bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6 && (topGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(topGlyphColor.getDistance(DistanceUnit.CM)))){
            glyphCase = 1;
            led.turnOn();

            //Raise lift to prevent glyph from dragging
            lift.setTargetPosition(800);
            lift.setPower(1);

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

            //Back up 10 inches
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 6*COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
            drive.resetEncoders();

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

            //Lift and flip
            spin.setPosition(SPIN_ROTATED);
            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());
            lift.setTargetPosition(5);
            lift.setPower(1);

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

            //Drive 20 inches forward to pick up second glyph
            while (drive.moveIMU(drive.getEncoderDistance(), 25 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0, 20 * COUNTS_PER_INCH, 1,
                    0.75, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

            boolean additionalDistance = false;
            //check if glyph in bottom arms of robot
            if (topGlyphColor.getDistance(DistanceUnit.CM)<=6){
                led.setLEDPower(1);
            }else{
                glyphWiggle(0, 45);
                timer.reset();
                while(timer.milliseconds()<250&&opModeIsActive());

                if(topGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(topGlyphColor.getDistance(DistanceUnit.CM))){
                    additionalDistance = true;
                }

            }
            drive.resetEncoders();

            if(additionalDistance){
                //Drive 6 inches forward to pick up second glyph
                while (drive.moveIMU(drive.getEncoderDistance(), 6 * COUNTS_PER_INCH, 6 * COUNTS_PER_INCH, 0, 6 * COUNTS_PER_INCH, 1,
                        0.75, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());

                if (topGlyphColor.getDistance(DistanceUnit.CM)<=6){
                    led.setLEDPower(1);
                }else{
                    glyphWiggle(0, 30);
                    timer.reset();
                    while(timer.milliseconds()<250&&opModeIsActive());
                }
            }

            drive.resetEncoders();

            lift.setTargetPosition(250);
            lift.setPower(1);

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

            if(additionalDistance){
                //Back up 16 inches to original position
                drive.resetEncoders();
                while (drive.moveIMU(drive.getEncoderDistance(), 21 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
                drive.resetEncoders();
            }else {
                //Back up 15 inches to original position
                drive.resetEncoders();
                while (drive.moveIMU(drive.getEncoderDistance(), 15 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
                drive.resetEncoders();
            }
        }
        //CASE 2: THE ROBOT HAS 1 GLYPH IN THE TOP, BUT NOTHING IN THE BOTTOM
        else if((bottomGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(bottomGlyphColor.getDistance(DistanceUnit.CM)))&& topGlyphColor.getDistance(DistanceUnit.CM) <= 6){
            glyphCase = 2;
            //Drive 10 inches forward to pick up second glyph
            while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, 1,
                    0.75, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
            drive.resetEncoders();

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

            //Check if the robot has a glyph in the bottom
            if(bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6){ //If there is a glyph in the bottom
                //Raise lift to prevent glyph from dragging
                lift.setTargetPosition(800);
                lift.setPower(1);

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());

                //Drive back 10 inches to return to original position
                while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, 1,
                        0.75, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                drive.resetEncoders();

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());

            }
            else{ //The robot does not have a glyph in the bottom
                //Wiggle with an offset of 15 degrees to try and get a glyph
                glyphWiggle(0, 25);
                drive.resetEncoders();

                boolean additionalDistance = false;

                if(bottomGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(bottomGlyphColor.getDistance(DistanceUnit.CM))){
                    additionalDistance = true;
                }

                if(additionalDistance){
                    //Drive 6 inches forward to pick up second glyph
                    while (drive.moveIMU(drive.getEncoderDistance(), 6 * COUNTS_PER_INCH, 6 * COUNTS_PER_INCH, 0, 6 * COUNTS_PER_INCH, 1,
                            0.75, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());

                    timer.reset();
                    while(timer.milliseconds() < 1000 && opModeIsActive());

                    if (bottomGlyphColor.getDistance(DistanceUnit.CM)<=6){
                        led.setLEDPower(1);
                    }else{
                        glyphWiggle(0, 30);
                        timer.reset();
                        while(timer.milliseconds()<250&&opModeIsActive());
                    }
                }

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());

                //Raise lift to prevent glyph from dragging
                lift.setTargetPosition(800);
                lift.setPower(1);

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());

                if(additionalDistance){
                    //Drive back 10 inches to return to original position
                    while (drive.moveIMU(drive.getEncoderDistance(), 16 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, 1,
                            0.75, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive())
                        ;
                    drive.resetEncoders();
                }else {
                    //Drive back 10 inches to return to original position
                    while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, 1,
                            0.75, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive())
                        ;
                    drive.resetEncoders();
                }

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());
            }

            //Check if there is a glyph in the bottom
            if(bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6){ //If there is a glyph in the bottom

                //Raise lift to prevent glyph from dragging
                lift.setTargetPosition(300);

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());

            }else{ //If there is only a glyph in the top

                //Lift and flip to deposit the glyph in the bottom half of the mechanism
                spin.setPosition(SPIN_ROTATED);
                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());
                lift.setTargetPosition(300);
            }
        }
        //CASE 3: THE ROBOT DOES NOT HAVE ANY GLYPHS
        else if((bottomGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(bottomGlyphColor.getDistance(DistanceUnit.CM))) && (topGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(topGlyphColor.getDistance(DistanceUnit.CM)))){
            glyphCase = 3;
            //Drive 10 inches forward to pick up second glyph
            while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, 1,
                    0.75, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
            drive.resetEncoders();

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

            //Check if the robot has a glyph in the bottom
            if(bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6){ //If there is a glyph in the bottom
                //Raise lift to prevent glyph from dragging
                lift.setTargetPosition(800);
                lift.setPower(1);

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());

                //Drive back 10 inches to return to original position
                while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, 1,
                        0.75, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                drive.resetEncoders();

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());

            }
            else{ //The robot does not have a glyph in the bottom
                //Wiggle with an offset of 15 degrees to try and get a glyph
                glyphWiggle(0, 30);
                drive.resetEncoders();

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());

                boolean additionalDistance = false;
                if(bottomGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(bottomGlyphColor.getDistance(DistanceUnit.CM))){
                    additionalDistance = true;
                    //Drive 6 more inches to get a glyph
                    while (drive.moveIMU(drive.getEncoderDistance(), 6 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, 1,
                            0.75, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                    drive.resetEncoders();
                }

                //Raise lift to prevent glyph from dragging
                lift.setTargetPosition(800);
                lift.setPower(1);

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());

                if(additionalDistance){
                    //Drive back 16 inches to return to original position
                    while (drive.moveIMU(drive.getEncoderDistance(), 16 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, 1,
                            0.75, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                    drive.resetEncoders();
                }else{
                    //Drive back 10 inches to return to original position
                    while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, 1,
                            0.75, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
                    drive.resetEncoders();
                }

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());
            }

            //Back up 10 inches
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 6*COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
            drive.resetEncoders();

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

            //Lift and flip
            spin.setPosition(SPIN_ROTATED);
            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());
            lift.setTargetPosition(5);
            lift.setPower(1);

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

            //Drive 20 inches forward to pick up second glyph
            while (drive.moveIMU(drive.getEncoderDistance(), 25 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0, 20 * COUNTS_PER_INCH, 1,
                    0.75, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

            boolean additionalDistance = false;
            //check if glyph in bottom arms of robot
            if (topGlyphColor.getDistance(DistanceUnit.CM)<=6){
                led.setLEDPower(1);
            }else{
                glyphWiggle(0, 45);
                timer.reset();
                while(timer.milliseconds()<250&&opModeIsActive());

                if(topGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(topGlyphColor.getDistance(DistanceUnit.CM))){
                    additionalDistance = true;
                }

            }
            drive.resetEncoders();

            if(additionalDistance){
                //Drive 6 inches forward to pick up second glyph
                while (drive.moveIMU(drive.getEncoderDistance(), 6 * COUNTS_PER_INCH, 6 * COUNTS_PER_INCH, 0, 6 * COUNTS_PER_INCH, 1,
                        0.75, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());

                timer.reset();
                while(timer.milliseconds() < 1000 && opModeIsActive());

                if (topGlyphColor.getDistance(DistanceUnit.CM)<=6){
                    led.setLEDPower(1);
                }else{
                    glyphWiggle(0, 30);
                    timer.reset();
                    while(timer.milliseconds()<250&&opModeIsActive());
                }
            }

            drive.resetEncoders();

            lift.setTargetPosition(250);
            lift.setPower(1);

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

            if(additionalDistance){
                //Back up 16 inches to original position
                drive.resetEncoders();
                while (drive.moveIMU(drive.getEncoderDistance(), 21 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
                drive.resetEncoders();
            }else {
                //Back up 15 inches to original position
                drive.resetEncoders();
                while (drive.moveIMU(drive.getEncoderDistance(), 15 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
                drive.resetEncoders();
            }
        }

        //Check if there is a glyph in the bottom
        if(bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6){ //If there is a glyph in the bottom

            //Raise lift to prevent glyph from dragging
            lift.setTargetPosition(300);

            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());

        }else{ //If there is only a glyph in the top

            //Lift and flip to deposit the glyph in the bottom half of the mechanism
            spin.setPosition(SPIN_ROTATED);
            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive());
            lift.setTargetPosition(300);
        }

        bottomIntake.stop();
        topIntake.stop();

        while(opModeIsActive()){
            telemetry.addData("Glyph Case", glyphCase);
            telemetry.addData("Top Glyph Distance", topGlyphColor.getDistance(DistanceUnit.CM));
            telemetry.addData("Bottom Glyph Distance", bottomGlyphColor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        String fileName = "autoSpinPosition.txt";
        File file = AppUtil.getInstance().getSettingsFile(fileName);
        float spinPosition = (float) spin.getPosition();
        ReadWriteFile.writeFile(file, String.valueOf(spinPosition));


    }

    public void initVuforia(){
        telemetry.addData("Init", "Starting Vuforia");
        telemetry.update();

        //Initialize camera hardware and parameters
        cam = android.hardware.Camera.open();
        android.hardware.Camera.Parameters p = cam.getParameters();
        p.setFlashMode(android.hardware.Camera.Parameters.FLASH_MODE_TORCH);
        cam.setParameters(p);
        cam.startPreview();

        //Initialize vuforia parameters
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATXxmRr/////AAAAGdfeAuU6SEoFkpmhG616inkbnBHHQ/Ti5DMPAVykTBdmQS8ImGtoIBRRuboa+oIyuvQW1nIychXXxROjGLssEzSFF8yOYE36GqhVtRfI6lw8/HAoJpO1XgIF5Gy1vPx4KFPNInK6CJdZomYyWV8rGnb7ceLJ9Z+g0sl+VcVPKl5DAI84K+06pEZnw+Em7sThhzyzj2p4QbPhXh7fEtNGhFCqey9rcg3h9RfNebyWvJW9z7mGkaJljZy1x3lK7viLbFKyFcAaspZZi1+JzUmeuXxV0r+8hrCgFLPsvKQHlnYumazP9FEtm/FjCpRFF23Et77325/vuD2LRSPzve9ef4zqe6MivrLs9s8lUgd7Eo9W";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData("Init", "Finished Starting Vuforia");
        telemetry.update();
    }

    /**
     * Hardware maps all motors, servos, and sensors
     */
    public void initHardwareMap(){

        //Hardware Map Motors
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        lift = hardwareMap.dcMotor.get("glyph_lift");
        relic_extension = hardwareMap.dcMotor.get("relic_extension");
        leds = hardwareMap.dcMotor.get("leds");

        //Group drive motors in Array List
        motors = new ArrayList<>();
        motors.add(rf);
        motors.add(rb);
        motors.add(lf);
        motors.add(lb);

        //Hardware Map Servos
        rightWheel1 = hardwareMap.crservo.get("right_glyph1");
        leftWheel1 = hardwareMap.crservo.get("left_glyph1");
        rightWheel2 = hardwareMap.crservo.get("right_glyph2");
        leftWheel2 = hardwareMap.crservo.get("left_glyph2");
        spin = hardwareMap.servo.get("spin_grip");
        relic_arm = hardwareMap.servo.get("relic_arm");
        relic_claw = hardwareMap.servo.get("relic_claw");
        relic_tilt = hardwareMap.servo.get("relic_tilt");
        pan = hardwareMap.servo.get("jewel_pan");
        tilt = hardwareMap.servo.get("jewel_tilt");

        //Hardware Map Color Sensors
        jewel_color = (LynxI2cColorRangeSensor) hardwareMap.get("jewel_color");
        bottom_color = (LynxI2cColorRangeSensor) hardwareMap.get("floor_color");
        bottomGlyphColor = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor1");
        topGlyphColor = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor2");

        //Hardware Map Range Sensors
        ultrasonic_jewel = (ModernRoboticsI2cRangeSensor) hardwareMap.get("jewel_us");
        ultrasonic_back = (ModernRoboticsI2cRangeSensor) hardwareMap.get("back_us");
        ultrasonic_front = (ModernRoboticsI2cRangeSensor) hardwareMap.get("bottom_front_us");
        ultrasonic_front_top = (ModernRoboticsI2cRangeSensor) hardwareMap.get("top_front_us");

        //Hardware Map Limit Switch
        glyphLimit = hardwareMap.digitalChannel.get("glyph_limit");


    }

    /**
     * Set the mode, zero power behaviors, and servo positions for motors and servos
     */
    public void setMotorBehaviors(){
        //Set motor behaviors
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        relic_extension.setDirection(DcMotorSimple.Direction.REVERSE);
        relic_extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relic_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set servo behaviors
        leftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set Servo Init Positions
        relic.setTiltPosition(1);
        relic_arm.setPosition(0);
        pan.setPosition(0.5);
        tilt.setPosition(1);
        spin.setPosition(SPIN_START);
    }

    /**
     * Using the range sensors and the color sensor, the robot will automatically detect which stone the robot is on
     * and select the corresponding autonomous program
     */
    public void autoSelectAlignment(){
        relicTrackables.activate();
        boolean selected = false, selecting = true, aligned = false;
        double jewelValue = 255, backValue = 255, frontValue = 255;

        while(!selected&&!isStopRequested()){
            //Check to see if selection process exit is triggered
            if(isStopRequested()){
                selected = true;
            }
            if (gamepad1.b || gamepad2.b){
                selected = true;
            }
            //If the robot is aligned, turn on the LEDs to signal the drivers
            if(aligned){
                led.turnOn();
            }else{
                led.turnOff();
            }

            //Get updated US Values
            double jewelValueTemp = ultrasonic_jewel.cmUltrasonic();
            double backValueTemp = ultrasonic_back.cmUltrasonic();
            double frontValueTemp = ultrasonic_front.cmUltrasonic();

            //Filter out 255 values
            if(jewelValueTemp != 255){
                jewelValue = jewelValueTemp;
            }
            if(backValueTemp != 255){
                backValue = backValueTemp;
            }
            if(frontValueTemp != 255){
                frontValue = frontValueTemp;
            }

            //Show new US values
            telemetry.addData("Jewel US", jewelValue);
            telemetry.addData("Back US", backValue);
            telemetry.addData("Front US", frontValue);

            //Determine which stone the robot is on
            if(selecting){

                if(floor_color.red() > 35 && floor_color.getHSV()[0] < 30){
                    if(frontValue > 60 && backValue <= 40){
                        autoProgram = "RedStone1";
                    }
                    else if(frontValue > 60 && backValue > 90){
                        autoProgram = "RedStone2";
                    }
                }else if (floor_color.getHSV()[0] > 200){
                    if(frontValue < 50 && backValue > 60){
                        autoProgram = "BlueStone1";
                    }
                    else if(frontValue > 100 && (backValue > 85 && backValue < 150)){
                        autoProgram = "BlueStone2";
                    }
                }
            }

            telemetry.addData("Auto Program Selected", autoProgram);


            //Check if robot is aligned using ultrasonic values
            //If robot is aligned, the LEDs will turn on
            if(autoProgram.equals("RedStone1")){
                if(jewelValue == 36 && backValue == 37){
                    aligned = true;
                    led.turnOn();
                }else{
                    aligned = false;
                    led.turnOff();
                }
            }
            else if(autoProgram.equals("RedStone2")){
                if(jewelValue == 36 && frontValue == 93){
                    aligned = true;
                    led.turnOn();
                }else{
                    aligned = false;
                    led.turnOff();
                }
            }
            else if(autoProgram.equals("BlueStone1")){
                if(jewelValue == 36 && frontValue == 38){
                    aligned = true;
                    led.turnOn();
                }else{
                    aligned = false;
                    led.turnOff();
                }
            }
            else if(autoProgram.equals("BlueStone2")){
                if(jewelValue == 36 && backValue == 93){
                    aligned = true;
                    led.turnOn();
                }else{
                    aligned = false;
                    led.turnOff();
                }
            }
            telemetry.addData("Aligned", aligned);

            //Display the override controls
            telemetry.addData("Red Stone 1", "DPad-Up");
            telemetry.addData("Red Stone 2", "DPad-Left");
            telemetry.addData("Blue Stone 1", "DPad-Down");
            telemetry.addData("Blue Stone 2", "DPad-Right");

            //Controls to override selection
            if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left){
                selecting = false;
                if(gamepad1.dpad_up){
                    autoProgram = "RedStone1";
                    alliance = RED;
                    selected = true;
                }else if(gamepad1.dpad_left){
                    autoProgram = "RedStone2";
                    alliance = RED;
                    selected = true;
                }else if(gamepad1.dpad_down){
                    autoProgram = "BlueStone1";
                    alliance = BLUE;
                    selected = true;
                }else if(gamepad1.dpad_right){
                    autoProgram = "BlueStone2";
                    alliance = BLUE;
                    selected = true;
                }
            }
            //Update US values and alignment status
            telemetry.update();
        }
        if(autoProgram.equals("RedStone1") || autoProgram.equals("RedStone2")){ alliance = RED;}
        else{ alliance = BLUE; }
        led.turnOff();
    }

    /**
     * Calibrate the IMU
     */
    public void initIMU(){
        //Calibrate IMU
        telemetry.addData("Init", "IMU Calibrating");
        telemetry.update();
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Init", "IMU Instantiated");
        telemetry.update();
    }

    /**
     * Wiggle the robot in the glyph pit to pick up a glyph
     * @param center The original position of the robot, and the position it will return to
     * @param offset The amount of degrees the robot would pivot in each direction to wiggle
     */
    public void glyphWiggle(double center, double offset){
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(drive.pivotIMU(center+15, -90, .3, .25, 5, 0, Direction.FASTEST)&&opModeIsActive());
        while(drive.pivotIMU(center-15, -90, .3, .25, 5, 0, Direction.FASTEST)&&opModeIsActive());
        while(drive.pivotIMU(center, -90, .3, DEFAULT_MIN_POWER, 2, 500, Direction.FASTEST)&&opModeIsActive());
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Intake glyphs from the glyph pit and return to the cryptobox, in a position to deposit the glyphs
     * @param forwardDistance Distance to travel towards the glyph pit
     * @param backupDistance Distance to travel back to the cryptobox from the glyph pit
     * @param backUpAngle The angle that the robot will follow when backing up to the cryptobox
     * @param forwardMin The minimum speed the robot will travel when approaching the glyph pit
     * @param backwardMin The minimum speed the robot will travel when approaching the cryptobox from the glyph pit
     */
    public void getGlyphs(double forwardDistance, double backupDistance, double backUpAngle, double forwardMin, double backwardMin){
        //intake glyphs
        bottomIntake.secureGlyph();
        topIntake.secureGlyph();

        //Lower lift to intake glyphs
        lift.setTargetPosition(5);
        lift.setPower(1);

        //Move into Glyph Pit
        drive.resetEncoders();
        while(drive.moveIMU(drive.getEncoderDistance(), forwardDistance, forwardDistance-16*COUNTS_PER_INCH, 0, forwardDistance-4, DEFAULT_MAX_POWER, forwardMin, -90, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

        //Wait for glyphs to come into the robot
        timer.reset();
        while(opModeIsActive()&&timer.milliseconds()<250){

        }

        //check if glyph in bottom arms of robot
        if (bottomGlyphColor.getDistance(DistanceUnit.CM)<=6){
            led.setLEDPower(1);
        }else{
            glyphWiggle(-90, 15);
            timer.reset();
            while(timer.milliseconds()<250&&opModeIsActive());
        }
        drive.resetEncoders();
        lift.setTargetPosition(300);

        //back up to cryptobox
        while(drive.moveIMU(drive.getEncoderDistance(), backupDistance, backUpAngle-20*COUNTS_PER_INCH, 0, backupDistance-10*COUNTS_PER_INCH, DEFAULT_MAX_POWER, backwardMin, backUpAngle, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

    }

    /**
     * Deposit glyphs in the robot's possesion into the glyph pit
     * @param depositDistance Distance the robot should travel to deposit the glyphs
     * @param comeBackDistance Distance the robot should travel after depositing the glyphs
     * @param depositAngle The angle the robot should move at when depositing a glyph in the pit
     * @param depositLiftPosition The position the lift should be in before depositing the glyph
     */
    public void depositGlyphs(double depositDistance, double comeBackDistance, double depositAngle, int depositLiftPosition){
        //Move lift in position to deposit glyphs
        lift.setTargetPosition(depositLiftPosition);

        //Pivot to face cryptobox at the desired angle
        while(drive.pivotIMU(depositAngle, -90, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 2, 500, Direction.FASTEST)&&opModeIsActive());
        drive.stop();

        //drive into cryptobox
        timer.reset();
        drive.resetEncoders();
        while(drive.moveIMU(drive.getEncoderDistance(), depositDistance, depositDistance-4*COUNTS_PER_INCH, 0, 2*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, depositAngle, DEFAULT_PID, depositAngle, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive()&&timer.milliseconds()<5000){

        }
        //deposit glyph
        bottomIntake.dispenseGlyph();
        topIntake.dispenseGlyph();
        timer.reset();
        while(timer.milliseconds()<250&&opModeIsActive());

        //lower the lift
        lift.setTargetPosition(depositLiftPosition-200);

        timer.reset();
        //push the glyph in another inch while ejecting
        drive.resetEncoders();
        while(drive.moveIMU(drive.getEncoderDistance(), 1*COUNTS_PER_INCH, 0, 0, .5*COUNTS_PER_INCH, DEFAULT_MIN_POWER, DEFAULT_MIN_POWER, depositAngle, DEFAULT_PID, depositAngle, DEFAULT_ERROR_DISTANCE, 0)&&opModeIsActive()&&timer.milliseconds()<1000){

        }
        drive.resetEncoders();

        //Back away from cryptobox
        while(drive.moveIMU(drive.getEncoderDistance(), comeBackDistance+1*COUNTS_PER_INCH, 4*COUNTS_PER_INCH, 0, 2*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, depositAngle+180, DEFAULT_PID, depositAngle, DEFAULT_ERROR_DISTANCE, 200)&&opModeIsActive());
        drive.resetEncoders();

    }
}
