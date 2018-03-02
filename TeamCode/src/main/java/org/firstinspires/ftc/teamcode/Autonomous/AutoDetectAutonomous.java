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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.MecanumDriveTrain;
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

    //create sensor interface variables
    BNO055IMU boschIMU;
    DualWheelIntake intake;
    ClawThreePoint relic;
    IColorSensor jewelColor;
    IColorSensor floor_color;
    TwoPointJewelArm jewel;
    IUltrasonic jewel_us;
    IUltrasonic back_us;
    LED led;

    //create vuforia variables
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    //create hardware variables
    Servo pan, tilt;
    CRServo rightWheel1, rightWheel2, leftWheel1, leftWheel2;
    Servo spin;
    DcMotor rf, rb, lf, lb;
    List<DcMotor> motors;
    DcMotor lift;
    DcMotor relic_extension;
    Servo relic_claw, relic_arm, relic_tilt;
    DigitalChannel glyphLimit;
    LynxI2cColorRangeSensor jewel_color, bottom_color, bottomGlyphColor, topGlyphColor;
    IIMU imu;
    MecanumDriveTrain drive;
    ModernRoboticsI2cRangeSensor ultrasonic_jewel;
    ModernRoboticsI2cRangeSensor ultrasonic_back;
    ModernRoboticsI2cRangeSensor ultrasonic_front, ultrasonic_front_top;
    DcMotor leds;

    //create timers and misc variables
    ElapsedTime timer;
    ElapsedTime gameTime;
    String vumarkSeen = "";
    double vuMarkDistance = 36;

    final double DEFAULT_MAX_POWER = .75;
    final double DEFAULT_MIN_POWER = .2;
    final double DEFAULT_ERROR_DISTANCE = 10;
    final double[] DEFAULT_PID = {.05};



    final double SPIN_START = 0;
    final double SPIN_ROTATED = .95;

    final double RELIC_CLAW_CLOSED = 1;
    final double RELIC_CLAW_OPENED = 0;

    final double RELIC_TILT_ORIGIN = 1;

    final double RELIC_ARM_ORIGIN = 0;

    final double COUNTS_PER_INCH = 45;
    final double CENTER_STONE_1_DIST = 31*COUNTS_PER_INCH;
    final double COLUMN_OFFSET = 7.5*COUNTS_PER_INCH;

    double imuAngle, encoderAverage, powerChange = 0;

    final int RED_LINE_COLOR_VALUE = 50;

    final double LIFT_POWER_UP = 1;
    final double LIFT_POWER_DOWN = -1;

    private final double INTAKE_SPPED = 0.74;
    private final double OUTTAKE_SPEED = -0.74;
    private final double REDUCED_OUTTAKE_SPEED = -0.25;

    boolean needToWiggle = false;

    String autoProgram = "";

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();      //initialize all hardware
        initVuforia();          //initialize vuforia

        //Initialize all objects before selecting autonomous
        floor_color = new LynxColorRangeSensor(bottom_color);
        led = new LED(leds);
        jewelColor = new LynxColorRangeSensor(jewel_color);
        jewel = new TwoPointJewelArm(pan, tilt, jewelColor, telemetry);
        intake = new DualWheelIntake(rightWheel1, rightWheel2, leftWheel1, leftWheel2, spin, lift, glyphLimit, telemetry);
        relic = new ClawThreePoint(relic_extension, relic_arm, relic_tilt, relic_claw);


        autoSelectAlignment();  //determine which autonomous to run
        setMotorBehaviors();    //set the motors to their correct modes
        initIMU();              //Initialize IMU after robot is set

        drive = new MecanumDriveTrain(motors, imu, telemetry);  //initialize drivetrain after IMU is reset

        //Init timers
        timer = new ElapsedTime();
        gameTime = new ElapsedTime();

        //Finish init
        telemetry.addData("Init", "Finished");
        telemetry.addData("Auto Program Selected", autoProgram);
        telemetry.update();
        led.setLEDPower(0.2);

        //Wait for program to start
        waitForStart();
/*
**************************************************************************************************************************************
***********************************************     OPMODE RUNS HERE     *************************************************************
***************************************************************************************************************************************
 */


        //Reset counters, timers, and activate Vuforia
        led.turnOff();
        relicTrackables.activate();
        timer.reset();
        gameTime.reset();
        //Reset drive encoders
        drive.resetEncoders();

        //Turn on the Intake and Raise the Lift to snap in arms the arms
        intake.secureGlyph();
        intake.setLiftTargetPosition(200, 1);

        //Move jewel to position and read color
        //jewel.setPanTiltPos(0.5, 0.21);
        timer.reset();
        while(timer.milliseconds() < 1000  && opModeIsActive()){
            telemetry.addData("Jewel", "Moving to Read Position");
            telemetry.addData("Timer", timer.milliseconds());
            telemetry.update();
        }

        //read jewel color and lift the lift even higher
        //jewel.readColor(5);
        intake.setLiftTargetPosition(700, 1);

        // Select which Jewel to knock off
        /*if(autoProgram.equals("RedStone1")||autoProgram.equals("RedStone2")){
            //Knock off jewel
            jewel.knockOffJewel("red");
        }else{
            jewel.knockOffJewel("blue");
        }*/

        //bring jewel back to starting position
        //jewel.setPanTiltPos(0.5, 1);
        //telemetry.addData("Jewel", "Done");

        //Read VuMark and determine drive distance and column
        //telemetry.addData("VuMark", "Reading");
        //telemetry.update();
        //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        /*if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            vumarkSeen = vuMark.toString();
            telemetry.addData("VuMark", vumarkSeen);
            telemetry.update();
        }*/

        //Move off the stone
        while(drive.moveIMU(drive.getEncoderDistance(), CENTER_STONE_1_DIST, 0, 0, 18*COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());
        /*while(drive.moveIMU(drive.getEncoderDistance(), CENTER_STONE_1_DIST, 0, 0, DEFAULT_MIN_POWER, DEFAULT_MIN_POWER, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 2000)&&opModeIsActive());

        //pivot to face cryptobox
        while(drive.pivotIMU(75, 30, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 1, 500, Direction.FASTEST)&&opModeIsActive());

        //lower lift to deposit position
        lift.setTargetPosition(344);
        lift.setPower(1);

        //drive into cryptobox
        drive.resetEncoders();
        while(drive.moveIMU(drive.getEncoderDistance(), 5*COUNTS_PER_INCH, 2.5*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 75, DEFAULT_PID, 75, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

        //dispense glyph
        intake.dispenseGlyph();

        //wait for glyph to go into cryptobox
        timer.reset();
        while(timer.milliseconds() < 250 && opModeIsActive()){

        }

        //Back away from cryptobox
        drive.resetEncoders();
        while(drive.moveIMU(drive.getEncoderDistance(), 6*COUNTS_PER_INCH, 3*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 255, DEFAULT_PID, 75, DEFAULT_ERROR_DISTANCE, 200)&&opModeIsActive());

        //Pivot to face Glyph Pit
        while(drive.pivotIMU(-90, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 2, 500, Direction.FASTEST)&&opModeIsActive());

        //intake glyphs
        intake.secureGlyph();

        //Lower lift to intake glyphs
        lift.setTargetPosition(5);
        lift.setPower(1);

        //Move into Glyph Pit
        drive.resetEncoders();
        while(drive.moveIMU(drive.getEncoderDistance(), 28*COUNTS_PER_INCH, 25*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, -90, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

        //Wait for glyphs to come into the robot
        timer.reset();
        while(opModeIsActive()&&timer.milliseconds()<250){

        };

        //check if glyph in bottom arms of robot
        if (bottomGlyphColor.getDistance(DistanceUnit.CM)<=6){
            led.setLEDPower(1);
        }else{
            glyphWiggle(-90, 15);
            timer.reset();
            while(timer.milliseconds()<250&&opModeIsActive());
        }
        drive.resetEncoders();
        lift.setTargetPosition(96);

        while(drive.moveIMU(drive.getEncoderDistance(), 26*COUNTS_PER_INCH, 0, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 90, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());
        if(bottomGlyphColor.getDistance(DistanceUnit.CM)<=6||topGlyphColor.getDistance(DistanceUnit.CM)<=6){
            lift.setTargetPosition(898);
            while(drive.pivotIMU(75, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 2, 500, Direction.FASTEST));

            //drive into cryptobox
            drive.resetEncoders();
            while(drive.moveIMU(drive.getEncoderDistance(), 6*COUNTS_PER_INCH, 2.5*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 75, DEFAULT_PID, 75, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());
            led.turnOff();

            //deposit glyph
            intake.dispenseGlyph();
            timer.reset();
            while(timer.milliseconds()<500&&opModeIsActive());

            //Back away from cryptobox
            drive.resetEncoders();
            while(drive.moveIMU(drive.getEncoderDistance(), 6*COUNTS_PER_INCH, 3*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 255, DEFAULT_PID, 75, DEFAULT_ERROR_DISTANCE, 200)&&opModeIsActive());

            while(drive.pivotIMU(-90, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 2, 500, Direction.FASTEST)&&opModeIsActive());
        }

        drive.resetEncoders();
        while(drive.moveIMU(drive.getEncoderDistance(), COLUMN_OFFSET, COLUMN_OFFSET, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 0, DEFAULT_PID, -90, 10, 500)&&opModeIsActive());

        //intake glyphs
        intake.secureGlyph();

        //Lower lift to intake glyphs
        lift.setTargetPosition(5);
        lift.setPower(1);

        //Move into Glyph Pit
        drive.resetEncoders();
        while(drive.moveIMU(drive.getEncoderDistance(), 28*COUNTS_PER_INCH, 25*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, -90, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

        //Wait for glyphs to come into the robot
        timer.reset();
        while(opModeIsActive()&&timer.milliseconds()<250){

        };

        //check if glyph in bottom arms of robot
        if (bottomGlyphColor.getDistance(DistanceUnit.CM)<=6){
            led.setLEDPower(1);
        }else{
            glyphWiggle(-90, 15);
            timer.reset();
            while(timer.milliseconds()<250&&opModeIsActive());
        }
        drive.resetEncoders();

        lift.setTargetPosition(192);
        while(drive.moveIMU(drive.getEncoderDistance(), 26*COUNTS_PER_INCH, 0, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 90, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

        if (bottomGlyphColor.getDistance(DistanceUnit.CM)<=6||topGlyphColor.getDistance(DistanceUnit.CM)<6) {
            while(drive.pivotIMU(75, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 2, 500, Direction.FASTEST));

            //drive into cryptobox
            drive.resetEncoders();
            while(drive.moveIMU(drive.getEncoderDistance(), 7*COUNTS_PER_INCH, 2.5*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 75, DEFAULT_PID, 75, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

            led.turnOff();

            //deposit glyph
            intake.dispenseGlyph();
            timer.reset();
            while(timer.milliseconds()<500&&opModeIsActive());

            //Back away from cryptobox
            drive.resetEncoders();
            while(drive.moveIMU(drive.getEncoderDistance(), 7*COUNTS_PER_INCH, 3*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 255, DEFAULT_PID, 75, DEFAULT_ERROR_DISTANCE, 200)&&opModeIsActive());
            drive.stop();
        }else{
            while(drive.moveIMU(drive.getEncoderDistance(), 32*COUNTS_PER_INCH, 14*COUNTS_PER_INCH, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 90, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

            drive.resetEncoders();
            while(drive.moveIMU(drive.getEncoderDistance(), COLUMN_OFFSET, COLUMN_OFFSET, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 0, DEFAULT_PID, -90, 10, 500)&&opModeIsActive());
            drive.stop();
        }
*/
        /*if(autoProgram.equals("RedStone1")){

            //Set the distances (in inches) to move based on the VuMark Target.
            // The distance from the wall is about 36 cm or 14 inches
            if(vumarkSeen.equals("LEFT")){
                vuMarkDistance = 29;
            }else if (vumarkSeen.equals("RIGHT")){
                vuMarkDistance = 12;
            }else {
                vuMarkDistance = 20;
            }
            //Drive to desired VuMark target
            //FIRST motion moving off the stone
            drive.resetEncoders();
            powerChange = (vuMarkDistance*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < vuMarkDistance*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.7, 0.3, powerChange, .15, 0, .008, 0.001, 0,
                        false, 1000);
                powerChange = (vuMarkDistance*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder(); // Do we need this????

            //Lower lift to dispense glyph
            lift.setTargetPosition(355);
            lift.setPower(1);

            //Pivot to face cryptobox
            while (drive.moveIMU(.7, 0.2, 0, 0, 0, 0, .008, 75, true, 500) && opModeIsActive()) {
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            powerChange = (2.5*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();
            while(drive.averageEncoders() < 2.5*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 1000){
                drive.moveIMU(0.3, 0.2, powerChange, .15, 90, .008, 0.001, 75,
                        false, 1000);
                powerChange = (2.5*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();

            //Outtake glyph into cryptobox
            intake.dispenseGlyph();
            timer.reset();

            while(timer.milliseconds() < 750 && opModeIsActive()){
                telemetry.addData("Intake", "Dispensing Glyph");
                telemetry.update();
            }

            //Strafe back away from cryptobox 6 Inches at orientation of 90 degrees
            drive.softResetEncoder();
            powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.6, 0.5, powerChange, .075, -90, 0.008, 0.001, 90,
                        false, 1000);
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            intake.setIntakePowerZero();

            //Strafing 3 inches to align with cryptobox center
            drive.softResetEncoder();
            powerChange = (3*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 3*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.6, 0.5, powerChange, .075, 0, 0.008, 0.001, 90,
                        false, 1000);
                powerChange = (3*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Pivot to face glyph pit
            while(drive.moveIMU(0.5, 0.3, 0, 0, 0, 0, 0.005, -90, true, 150) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();

            //Lower lift to intake glyphs
            lift.setTargetPosition(5);
            lift.setPower(1);

            //Drive to glyph pit and intake glyph
            intake.secureGlyph();
            powerChange = (20*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();

            //go into the pit at 20 inches
            drive.softResetEncoder();
            while(drive.averageEncoders()<20*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<5000){
                drive.moveIMU(.7, 0.35, powerChange, .035, -90, .02, 0.001, -90,
                        false, 1000);
                powerChange = (20*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            timer.reset();

            //wait for glyphs to come in robot
            while(opModeIsActive()&&timer.milliseconds()<750);


            //check if glyph in bottom arms of robot
            if (bottomGlyphColor.getDistance(DistanceUnit.CM)<6){
                additionalDistance = false;
                led.setLEDPower(1);
            }else{
                additionalDistance = true;
            }


            //Go to 24 farther into glyph pit if glyph is not in the bottom
            if (additionalDistance) {
                powerChange = (24*COUNTS_PER_INCH) - drive.averageEncoders();
                timer.reset();
                while(drive.averageEncoders()<24*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<4000){
                    drive.moveIMU(.7, .7, powerChange, .175, -90, 0.008, 0.001, -90,
                            false, 1000);
                    powerChange = (24*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                drive.setPowerZero();
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
            lift.setTargetPosition(898);
            lift.setPower(1);
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();

            //Drive back to cryptobox at 12 or 16 inches

            if(additionalDistance){
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 3000){
                    drive.moveIMU(.4, 0.15, powerChange, .04, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();

                }
                powerChange = (16*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 16*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (16*COUNTS_PER_INCH) - drive.averageEncoders();
                }

            }else{
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                    drive.moveIMU(.4, 0.15, powerChange, .07, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 12*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
                }

            }


            led.setLEDPower(0);

            drive.setPowerZero();
            drive.softResetEncoder();

            //Check if we have glyph
            boolean hasGlyph;
            if(bottomGlyphColor.getDistance(DistanceUnit.CM) < 7 || topGlyphColor.getDistance(DistanceUnit.CM) < 7){
                hasGlyph = true;
            }else{
                hasGlyph = false;
            }
            //Only go to cryptobox if there is a glyph
            if(hasGlyph){
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
                while(drive.averageEncoders() < 12*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 4000){
                    drive.moveIMU(.7, 0.1, powerChange, .045, 90, .04, .043, 90,
                            false, 1000);
                    powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                drive.softResetEncoder();
                drive.setPowerZero();

                //Dispense glyph
                intake.dispenseGlyph();
                timer.reset();
                while(timer.milliseconds() < 500  && opModeIsActive()){
                    telemetry.addData("Intake", "Dispensing Glyph");
                    telemetry.update();
                }

                //Back away from cryptobox
                powerChange = (12 *COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 12*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.3, 0.1, powerChange, .03, -90, 0.015, 0.001, 90,
                            false, 1000);
                    powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
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

            }else{
                //Lower lift to intake glyphs
                lift.setTargetPosition(5);
                lift.setPower(1);
                timer.reset();
                while(timer.milliseconds() < 250 && opModeIsActive()){

                }
            }
            //Slide to new column
            drive.softResetEncoder();
            if(!vumarkSeen.equals("RIGHT")){
                powerChange = (6.5*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 6.5*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(1, 1, powerChange, .125, 180, .02, 0.001, -90,
                            false, 1000);
                    powerChange = (6.5*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }else{
                powerChange = (6.5*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 6.5*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(1, 1, powerChange, .125, 0, .02, 0.001, -90,
                            false, 1000);
                    powerChange = (6.5*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //turn in intake
            intake.secureGlyph();

            //go into glyph pit  at 14 inches

            powerChange = (14*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();
            while(drive.averageEncoders()<14*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<5000) {
                drive.moveIMU(.7, 0.35, powerChange, .035, -90, .02, 0.001, -90,
                        false, 1000);
                powerChange = (14 * COUNTS_PER_INCH) - drive.averageEncoders();
            }


            drive.setPowerZero();
            timer.reset();

            //wait for glyphs to come in robot
            while(opModeIsActive()&&timer.milliseconds()<500);


            //check if glyph in bottom arms of robot
            if (bottomGlyphColor.getDistance(DistanceUnit.CM)<6){
                additionalDistance = false;
                led.setLEDPower(1);
            }else{
                additionalDistance = true;
            }

            //Go get farther into pit if glyph is not in the bottom
            if (additionalDistance) {
                powerChange = (18*COUNTS_PER_INCH) - drive.averageEncoders();
                timer.reset();
                while(drive.averageEncoders()<18*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<2000){
                    drive.moveIMU(.7, .7, powerChange, .175, -90, 0.008, 0.001, -90,
                            false, 1000);
                    powerChange = (18*COUNTS_PER_INCH) - drive.averageEncoders();
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
            lift.setTargetPosition(336);
            lift.setPower(1);
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();

            //Drive back to cryptobox at 8 or 12 inches

            if(additionalDistance){
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 3000){
                    drive.moveIMU(.4, 0.15, powerChange, .04, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 12*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
                }

            }else{
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                    drive.moveIMU(.4, 0.15, powerChange, .07, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                powerChange = (8*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 8*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (8*COUNTS_PER_INCH) - drive.averageEncoders();
                }

            }

            lift.setTargetPosition(432);
            lift.setPower(1);
            drive.softResetEncoder();
            led.setLEDPower(0);

            if(bottomGlyphColor.getDistance(DistanceUnit.CM) < 6 || topGlyphColor.getDistance(DistanceUnit.CM) < 6){
                hasGlyph = true;
            }else{
                hasGlyph = false;
            }



            if(!hasGlyph){
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .1, .008, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                drive.setPowerZero();
                lift.setTargetPosition(5);
                while(opModeIsActive());
            }


            //Pivot to face cryptobox
            while(drive.moveIMU(0.5, 0.2, 0, 0, 0, 0, 0.005, 90, true, 500) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.softResetEncoder();
            intake.setIntakePowerZero();

            //Drive to cryptobox
            powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();
            while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                drive.moveIMU(.7, 0.1, powerChange, .4375, 90, 0.008, .008, 90,
                        false, 1000);
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.softResetEncoder();
            drive.setPowerZero();

            //Dispense glyph
            if(vumarkSeen.equals("CENTER") || vumarkSeen.equals("")){
                intake.setIntakePower(OUTTAKE_SPEED, REDUCED_OUTTAKE_SPEED);
            }else if(vumarkSeen.equals("LEFT")){
                intake.dispenseGlyph();
            }else if(vumarkSeen.equals("RIGHT")){
                intake.dispenseGlyph();
            }else{
                intake.dispenseGlyph();
            }
            timer.reset();
            while(timer.milliseconds() < 1000 && opModeIsActive()){
                telemetry.addData("Intake", "Dispensing Glyph");
                telemetry.update();
            }
            //Back away from cryptobox
            powerChange = (5*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 5*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.3, 0.1, powerChange, .15, -90, 0.008, 0.001, 90,
                        false, 1000);
                powerChange = (5*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.softResetEncoder();
            drive.setPowerZero();
            intake.setIntakePowerZero();

            //Lower lift to intake glyphs
            lift.setTargetPosition(5);
            lift.setPower(1);
            while(opModeIsActive());

        }else if (autoProgram.equals("RedStone2")){
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
            lift.setTargetPosition(240);
            lift.setPower(1);
            timer.reset();


            //Drive to park zone
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
                lift.setTargetPosition(864);
            }else{
                lift.setTargetPosition(144);
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
            //Determine VuMark distances
            if(vumarkSeen.equals("LEFT")){
                vuMarkDistance = 14;
            }else if (vumarkSeen.equals("RIGHT")){
                vuMarkDistance = 30;
            }else {
                vuMarkDistance = 22;
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
            lift.setTargetPosition(355);
            lift.setPower(1);

            //Pivot to face cryptobox

            while(drive.moveIMU(.7, 0.2, 0, 0, 0, 0, .008, 105, true, 500) && opModeIsActive()){
                telemetry.addData("Move", "Pivot");
                telemetry.update();
            }
            drive.setPowerZero();
            drive.softResetEncoder();

            //Drive to glyph release location
            powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();
            while(drive.averageEncoders() < 4*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 3000){
                drive.moveIMU(0.3, 0.2, powerChange, .15, 90, .008, 0.001, 105,
                        false, 1000);
                powerChange = (4*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();


            //Outtake glyph into cryptobox
            intake.dispenseGlyph();


            timer.reset();
            while(timer.milliseconds() < 750 && opModeIsActive()){
                telemetry.addData("Intake", "Dispensing Glyph");
                telemetry.update();
            }

            //Back away from cryptobox
            powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 6*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.6, 0.5, powerChange, .075, -90, 0.008, 0.001, 105,
                        false, 1000);
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            intake.setIntakePowerZero();

            //Strafe to center with cryptobox column
            powerChange = (3*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 3*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.6, 0.5, powerChange, .075, 180, 0.008, 0.001, 105,
                        false, 1000);
                powerChange = (3*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            drive.softResetEncoder();


            //Pivot to face glyph pit
            while(drive.moveIMU(0.5, 0.3, 0, 0, 0, 0, 0.005, -90, true, 500) && opModeIsActive()){
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

            //go into the glyph pit
            while(drive.averageEncoders()<20*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<5000){
                drive.moveIMU(.7, 0.35, powerChange, .035, -90, .02, 0.001, -90,
                        false, 1000);
                powerChange = (20*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
            timer.reset();

            //wait for glyphs to come in robot
            while(opModeIsActive()&&timer.milliseconds()<750);


            //check if glyph in bottom arms of robot
            if (bottomGlyphColor.getDistance(DistanceUnit.CM)<6){
                additionalDistance = false;
                led.setLEDPower(1);
            }else{
                additionalDistance = true;
            }

            //Go get farther into glyph pit if glyph is not in the bottom

            if (additionalDistance) {
                timer.reset();
                powerChange = (24*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<24*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<2000){
                    drive.moveIMU(.7, .7, powerChange, .075, -90, 0.008, 0.001, -90,
                            false, 1000);
                    powerChange = (24*COUNTS_PER_INCH) - drive.averageEncoders();
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
            lift.setTargetPosition(898);
            lift.setPower(1);
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();
            //Drive back to cryptobox at 12 or 16 inches

            if(additionalDistance){
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 3000){
                    drive.moveIMU(.4, 0.15, powerChange, .04, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();

                }
                powerChange = (16*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 16*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (16*COUNTS_PER_INCH) - drive.averageEncoders();
                }

            }else{
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 2000){
                    drive.moveIMU(.4, 0.15, powerChange, .07, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 12*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
                }

            }

            drive.setPowerZero();
            drive.softResetEncoder();

            //Check if we have glyph
            boolean hasGlyph;
            if(bottomGlyphColor.getDistance(DistanceUnit.CM) < 6 || topGlyphColor.getDistance(DistanceUnit.CM) < 6){
                hasGlyph = true;
            }else{
                hasGlyph = false;
            }

            if(hasGlyph){
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
                while(drive.averageEncoders() < 12*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 4000){
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
                powerChange = (12 *COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 12*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.3, 0.1, powerChange, .03, -90, 0.008, 0.001, 90,
                            false, 1000);
                    powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
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
            }else{
                //Lower lift to intake glyphs
                lift.setTargetPosition(5);
                lift.setPower(1);
                timer.reset();
                while(timer.milliseconds() < 250 && opModeIsActive()){

                }
            }

            //Slide to new column
            if(!vumarkSeen.equals("RIGHT")){
                powerChange = (6.5*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 6.5*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(1, 1, powerChange, .125, 180, .02, 0.001, -90,
                            false, 1000);
                    powerChange = (6.5*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }else{
                powerChange = (6.5*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 6.5*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(1, 1, powerChange, .125, 0, .02, 0.001, -90,
                            false, 1000);
                    powerChange = (6.5*COUNTS_PER_INCH) - drive.averageEncoders();
                }
            }
            drive.setPowerZero();
            drive.softResetEncoder();
            //Drive to glyph pit and intake glyph
            intake.secureGlyph();
            powerChange = (14*COUNTS_PER_INCH) - drive.averageEncoders();
            timer.reset();

            //go into the cryptobox
            while(drive.averageEncoders()<14*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<5000){
                drive.moveIMU(.7, 0.35, powerChange, .035, -90, .02, 0.001, -90,
                        false, 1000);
                powerChange = (14*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.setPowerZero();
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
                powerChange = (18*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<18*COUNTS_PER_INCH&&opModeIsActive()&&timer.milliseconds()<2000){
                    drive.moveIMU(.3, .25, powerChange, .075, -90, 0.008, 0.001, -90,
                            false, 1000);
                    powerChange = (18*COUNTS_PER_INCH) - drive.averageEncoders();
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
            lift.setTargetPosition(144);
            lift.setPower(1);
            drive.setPowerZero();
            drive.softResetEncoder();
            timer.reset();

            //Drive back to cryptobox

            if(additionalDistance){
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 4000){
                    drive.moveIMU(.4, 0.15, powerChange, .04, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                //go back faster
                powerChange = (16*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 16*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .1, .008, -90,
                            false, 1000);
                    powerChange = (16*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                drive.setPowerZero();
                drive.softResetEncoder();
            }else{
                powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders()<6*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 3000){
                    drive.moveIMU(.4, 0.15, powerChange, .07, 90, .04, 0.001, -90,
                            false, 1000);
                    powerChange = (6*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                //go back faster
                powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 12*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .1, .008, -90,
                            false, 1000);
                    powerChange = (12*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                drive.setPowerZero();
                drive.softResetEncoder();
            }

            drive.softResetEncoder();
            led.setLEDPower(0);

            if (bottomGlyphColor.getDistance(DistanceUnit.CM) > 6 && topGlyphColor.getDistance(DistanceUnit.CM)>6){
                //go back faster
                powerChange = (2*COUNTS_PER_INCH) - drive.averageEncoders();
                while(drive.averageEncoders() < 2*COUNTS_PER_INCH && opModeIsActive()){
                    drive.moveIMU(0.6, 0.5, powerChange, .01, 90, .1, .008, -90,
                            false, 1000);
                    powerChange = (2*COUNTS_PER_INCH) - drive.averageEncoders();
                }
                drive.setPowerZero();
                drive.softResetEncoder();
                while(opModeIsActive());
            }


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
            while(drive.averageEncoders() < 12*COUNTS_PER_INCH && opModeIsActive() && timer.milliseconds() < 4000){
                drive.moveIMU(.7, 0.1, powerChange, .4375, 90, 0.008, .008, 90,
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
            drive.softResetEncoder();
            //Back away from cryptobox
            powerChange = (5*COUNTS_PER_INCH) - drive.averageEncoders();
            while(drive.averageEncoders() < 5*COUNTS_PER_INCH && opModeIsActive()){
                drive.moveIMU(0.2, 0.2, powerChange, .15, -90, 0.008, 0.001, 90,
                        false, 1000);
                powerChange = (5*COUNTS_PER_INCH) - drive.averageEncoders();
            }
            drive.softResetEncoder();
            drive.setPowerZero();
            intake.setIntakePowerZero();

            //Lower lift to intake glyphs
            lift.setTargetPosition(5);
            lift.setPower(1);
            while(opModeIsActive());

        }else if (autoProgram.equals("BlueStone2")){
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
            lift.setTargetPosition(140);
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
                lift.setTargetPosition(864);
            }else{
                lift.setTargetPosition(144);
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

        }*/

    }

    public void initVuforia(){
        telemetry.addData("Init", "Starting Vuforia");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ATXxmRr/////AAAAGdfeAuU6SEoFkpmhG616inkbnBHHQ/Ti5DMPAVykTBdmQS8ImGtoIBRRuboa+oIyuvQW1nIychXXxROjGLssEzSFF8yOYE36GqhVtRfI6lw8/HAoJpO1XgIF5Gy1vPx4KFPNInK6CJdZomYyWV8rGnb7ceLJ9Z+g0sl+VcVPKl5DAI84K+06pEZnw+Em7sThhzyzj2p4QbPhXh7fEtNGhFCqey9rcg3h9RfNebyWvJW9z7mGkaJljZy1x3lK7viLbFKyFcAaspZZi1+JzUmeuXxV0r+8hrCgFLPsvKQHlnYumazP9FEtm/FjCpRFF23Et77325/vuD2LRSPzve9ef4zqe6MivrLs9s8lUgd7Eo9W";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData("Init", "Finished Starting Vuforia");
        telemetry.update();
    }

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
    }

    public void autoSelectAlignment(){
        boolean selected = false;
        boolean selecting = true;
        boolean aligned = false;
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
            telemetry.addData("Red Stone 1", "DPad-Up");
            telemetry.addData("Red Stone 2", "DPad-Left");
            telemetry.addData("Blue Stone 1", "DPad-Down");
            telemetry.addData("Blue Stone 2", "DPad-Right");

            if(jewelValue != 255 && backValue != 255 && selecting){

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

            if(autoProgram.equals("RedStone2") && jewelValue == 36 && backValue > 75 && frontValue == 93 && selecting){
                aligned = true;
                led.turnOn();
            }else if(autoProgram.equals("BlueStone2") && jewelValue == 36 && frontValue > 75 && backValue == 93 && selecting){
                aligned = true;
                led.turnOn();
            }else if (autoProgram.equals("BlueStone2") || autoProgram.equals("RedStone2") && selecting){
                aligned = false;
                led.turnOff();
            }

            if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left){
                selecting = false;
                if(gamepad1.dpad_up){
                    autoProgram = "RedStone1";
                    selected = true;
                }else if(gamepad1.dpad_left){
                    autoProgram = "RedStone2";
                    selected = true;
                }else if(gamepad1.dpad_down){
                    autoProgram = "BlueStone1";
                    selected = true;
                }else if(gamepad1.dpad_right){
                    autoProgram = "BlueStone2";
                    selected = true;
                }
            }
            if(backValue==255 || jewelValue == 255){

            }else{
                telemetry.update();
            }
        }
        led.turnOff();
    }

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
    public void glyphWiggle(double center, double offset){
        while(drive.pivotIMU(center+15, -90, .3, DEFAULT_MIN_POWER, 5, 0, Direction.FASTEST)&&opModeIsActive());
        while(drive.pivotIMU(center-15, -90, .3, DEFAULT_MIN_POWER, 5, 0, Direction.FASTEST)&&opModeIsActive());
        while(drive.pivotIMU(center, -90, .3, DEFAULT_MIN_POWER, 2, 500, Direction.FASTEST)&&opModeIsActive());
    }
}