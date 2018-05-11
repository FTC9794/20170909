package org.firstinspires.ftc.teamcode.SampleTestCode;

import android.graphics.Camera;

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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Enums.Alliance;
import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntakeThread;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.IGlyph;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Jewel.TwoPointJewelArm;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.IUltrasonic;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation.TAG;
import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.Enums.Alliance.RED;
import static org.firstinspires.ftc.teamcode.Enums.Alliance.UNKNOWN;

/**
 * Created by Sarthak on 2/7/2018.
 */
@Autonomous(name = "V2 Auto Select Autonomous", group = "Autonomous")
public class FarStoneFlipAutonomous extends LinearOpMode {
    String fileName = "autoSpinPosition.txt";
    File file = AppUtil.getInstance().getSettingsFile(fileName);

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
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
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
    final double[] DEFAULT_PID_STRAFE = {.03};
    final double DEFAULT_MIN_POWER_PIVOT = .15;

    //define constants for servo positions
    final double SPIN_START = .91;
    final double SPIN_ROTATED = 0.09;

    final double RELIC_CLAW_CLOSED = 1;
    final double RELIC_CLAW_OPENED = 0;

    final double RELIC_TILT_ORIGIN = 1;

    final double RELIC_ARM_ORIGIN = 0;

    //define constants for distance movements using encoders
    final double COUNTS_PER_INCH = 45;
    final double CENTER_STONE_1_DIST = 32*COUNTS_PER_INCH;
    final double COLUMN_OFFSET = 7.5*COUNTS_PER_INCH;
    final double BLUE_ALLIANCE_OFFSET = 1.5*COUNTS_PER_INCH;

    //define variables to hold information regarding autonomous selection
    String autoProgram = "";
    Alliance alliance = UNKNOWN;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();      //initialize all hardware
        initVuforia();          //initialize vuforia

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
        telemetry.addData("Alliance", alliance);
        telemetry.update();
        led.setLEDPower(0.2);

        //Wait for program to start
        waitForStart();
        bottomIntakeThread.start();
        topIntakeThread.start();
        /*
         **************************************************************************************************************************************
         ***********************************************     OPMODE RUNS HERE     *************************************************************
         ***************************************************************************************************************************************
         */


        //Turn off the LEDs
        led.turnOff();
        //Reset timers
        timer.reset();
        gameTime.reset();

        //Reset drive encoders
        drive.resetEncoders();

        //Read VuMark and determine drive distance and column
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            vumarkSeen = vuMark.toString();
        }

        //If the robot did not detect the VuMark, turn on the camera's flash for a second reading
        if(vumarkSeen.equals("")){
            com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);
        }

        //Turn on the Intake and Raise the Lift to snap in arms the arms
        bottomIntake.secureGlyph();
        lift.setTargetPosition(250);
        lift.setPower(1);

        //Move jewel to position
        jewel.setPanTiltPos(0.5, 0.21);
        timer.reset();
        //Wait for servo to get into position
        while(timer.milliseconds() < 1000  && opModeIsActive()){
            bottomIntake.secureGlyph();
            telemetry.addData("Jewel", "Moving to Read Position");
            telemetry.addData("Timer", timer.milliseconds());
            telemetry.update();
        }

        //read jewel color and lift the lift even higher
        jewel.readColor(5);
        lift.setTargetPosition(700);
        lift.setPower(1);

        // Select which Jewel to knock off and at which speed to knock the jewel off
        if(autoProgram.equals("RedStone1")){
            while(!jewel.knockOffJewel(RED, false, true)){
                bottomIntake.secureGlyph();
            }
        }else if(autoProgram.equals("RedStone2")){
            while(!jewel.knockOffJewel(RED, true, false)){
                bottomIntake.secureGlyph();
            }
        }else if(autoProgram.equals("BlueStone1")){
            while(!jewel.knockOffJewel(BLUE, true, false)){
                bottomIntake.secureGlyph();
            }
        }else if(autoProgram.equals("BlueStone2")){
            while(!jewel.knockOffJewel(BLUE, true, true)){
                bottomIntake.secureGlyph();
            }
        }

        //bring jewel back to starting position
        jewel.setPanTiltPos(0.5, 1);

        //Read the vumark a second time, if not detected the first time
        if(vumarkSeen.equals("")) {
            //Read VuMark and determine drive distance and column
            vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                vumarkSeen = vuMark.toString();
            }
        }

        //Turn on the flash and deactivate vuforia
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(false);
        relicTrackables.deactivate();
        cam.stopPreview();

        /*
         **************************************************************************************************************************************
         ***********************************************     RED STONE 1     *************************************************************
         ***************************************************************************************************************************************
         */
        if(autoProgram.equals("RedStone1")){
            //check vumark and drive to the target cryptobox column
            while(drive.moveIMU(drive.getEncoderDistance(), 28*COUNTS_PER_INCH, 0, 0, 18*COUNTS_PER_INCH, .5, DEFAULT_MIN_POWER, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250)&&opModeIsActive());

            //Deposit preloaded glyph
            if(vumarkSeen.equals("LEFT")){
                depositGlyphs(7 * COUNTS_PER_INCH, 7 * COUNTS_PER_INCH, 55, 500, true);
            }else if(vumarkSeen.equals("RIGHT")){
                depositGlyphs(5 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 100, 500, true);
            }else{
                depositGlyphs(6 * COUNTS_PER_INCH, 6 * COUNTS_PER_INCH, 75, 500, true);
            }
            lift.setTargetPosition(898);
            lift.setPower(1);

            //Pivot to face glyph pit
            while(drive.pivotIMU(-90, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 500, Direction.FASTEST)&&opModeIsActive());

            lift.setTargetPosition(5);
            lift.setPower(1);
            topIntake.secureGlyph();
            bottomIntake.secureGlyph();

            //Get glyphs from pit
            drive.resetEncoders();
            while(drive.moveIMU(drive.getEncoderDistance(), 18*COUNTS_PER_INCH, 10*COUNTS_PER_INCH, 0, 5*COUNTS_PER_INCH, DEFAULT_MAX_POWER, .4, -90, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

            getGlyphsFarStone(-90, -90, 12);

            //Get glyphs from pit
            drive.resetEncoders();
            while(drive.moveIMU(drive.getEncoderDistance(), 16*COUNTS_PER_INCH, 10*COUNTS_PER_INCH, 0, 5*COUNTS_PER_INCH, DEFAULT_MAX_POWER, .4, 90, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

            // We want to spin the glyph depending on what is in the intake.
            // Condition 1: Regular starting position and top glyph and bottom is not present, spin
            if(spin.getPosition() > .5 && topGlyphColor.getDistance(DistanceUnit.CM) <= 6 && !(bottomGlyphColor.getDistance(DistanceUnit.CM)<=6)){
                //Move to Spin Position
                spin.setPosition(SPIN_ROTATED);
                timer.reset();
                while(timer.milliseconds()<500&&opModeIsActive());

                // Condition 2: Flipped position and bottom (red) glyph and top is not present, spin.
            }else if(spin.getPosition() <.5 && bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6 && !(topGlyphColor.getDistance(DistanceUnit.CM)<=6)){
                //Move to Spin Position
                spin.setPosition(SPIN_START);
                timer.reset();
                while(timer.milliseconds()<500&&opModeIsActive());
            }

            //Deposit additional glyphs
            if(vumarkSeen.equals("LEFT")){
                depositGlyphs(8 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 105, 500, false);
            }else if(vumarkSeen.equals("RIGHT")){
                depositGlyphs(8 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 60, 500, false);
            }else{
                depositGlyphs(8 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 105, 500, false);
            }
            if(gameTime.seconds()<28){
                while(drive.pivotIMU(-90, 0, .5, .25, 2, 500, Direction.FASTEST)&&opModeIsActive());
            }
            drive.stop();
            /*
             **************************************************************************************************************************************
             ***********************************************     BLUE STONE 1     *************************************************************
             ***************************************************************************************************************************************
             */
        }else if(autoProgram.equals("BlueStone1")){
            //check vumark and drive to the target cryptobox column
            while(drive.moveIMU(drive.getEncoderDistance(), 33*COUNTS_PER_INCH, 0, 0, 18*COUNTS_PER_INCH, .5, DEFAULT_MIN_POWER, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250)&&opModeIsActive());

            //Deposit preloaded glyph
            if(vumarkSeen.equals("LEFT")){
                depositGlyphs(6 * COUNTS_PER_INCH, 6 * COUNTS_PER_INCH, 75, 500, true);
            }else if(vumarkSeen.equals("RIGHT")){
                depositGlyphs(9 * COUNTS_PER_INCH, 9 * COUNTS_PER_INCH, 120, 500, true);
            }else{
                depositGlyphs(6 * COUNTS_PER_INCH, 6 * COUNTS_PER_INCH, 105, 500, true);
            }
            lift.setTargetPosition(898);
            lift.setPower(1);

            //Pivot to face glyph pit
            while(drive.pivotIMU(-90, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 500, Direction.FASTEST)&&opModeIsActive());

            lift.setTargetPosition(5);
            lift.setPower(1);
            topIntake.secureGlyph();
            bottomIntake.secureGlyph();

            //Get glyphs from pit
            drive.resetEncoders();
            while(drive.moveIMU(drive.getEncoderDistance(), 16*COUNTS_PER_INCH, 10*COUNTS_PER_INCH, 0, 5*COUNTS_PER_INCH, DEFAULT_MAX_POWER, .4, -90, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

            getGlyphsFarStone(-90, -90, 12);

            //Get glyphs from pit
            drive.resetEncoders();
            while(drive.moveIMU(drive.getEncoderDistance(), 14*COUNTS_PER_INCH, 10*COUNTS_PER_INCH, 0, 5*COUNTS_PER_INCH, DEFAULT_MAX_POWER, .4, 90, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

            // We want to spin the glyph depending on what is in the intake.
            // Condition 1: Regular starting position and top glyph and bottom is not present, spin
            if(spin.getPosition() > .5 && topGlyphColor.getDistance(DistanceUnit.CM) <= 6 && !(bottomGlyphColor.getDistance(DistanceUnit.CM)<=6)){
                //Move to Spin Position
                spin.setPosition(SPIN_ROTATED);
                timer.reset();
                while(timer.milliseconds()<500&&opModeIsActive());

                // Condition 2: Flipped position and bottom (red) glyph and top is not present, spin.
            }else if(spin.getPosition() <.5 && bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6 && !(topGlyphColor.getDistance(DistanceUnit.CM)<=6)){
                //Move to Spin Position
                spin.setPosition(SPIN_START);
                timer.reset();
                while(timer.milliseconds()<500&&opModeIsActive());
            }

            //Deposit additional glyphs
            if(vumarkSeen.equals("LEFT")){
                depositGlyphs(11 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 115, 500, false);
            }else if(vumarkSeen.equals("RIGHT")){
                depositGlyphs(11 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 75, 500, false);
            }else{
                depositGlyphs(11 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 75, 500, false);
            }
            if(gameTime.seconds()<28){
                while(drive.pivotIMU(-90, 0, .5, .25, 2, 500, Direction.FASTEST)&&opModeIsActive());
            }
            drive.stop();
        }
        /*
         **************************************************************************************************************************************
         ***********************************************     RED STONE 2     *************************************************************
         ***************************************************************************************************************************************
         */
        else if(autoProgram.equals("RedStone2")) {
            //Drive off of balancing stone
            while (drive.moveIMU(drive.getEncoderDistance(), 23 * COUNTS_PER_INCH, 0, 0, 14 * COUNTS_PER_INCH, .5,
                    DEFAULT_MIN_POWER, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive());
            drive.resetEncoders();
            drive.stop();

            //initialize ultrasonic variables
            int i = 0;
            double ultrasonicSum=0;
            double ultrasonicReading;

            //get 5 readings of the ultrasonic sensor and sum them
            while(opModeIsActive()&&i<5){
                ultrasonicReading = ultrasonic_jewel.cmUltrasonic();
                timer.reset();
                while(ultrasonicReading==255&&opModeIsActive()&&timer.seconds()<1000){
                    ultrasonicReading=ultrasonic_jewel.cmUltrasonic();
                }
                ultrasonicSum+=ultrasonicReading;
                i++;
            }

            //determine the average and correction due to ultrasonic sensor readings
            double ultrasonicAverage = ultrasonicSum/i;
            double ultrasonicCorrection = (ultrasonicAverage-36)/2.54;

            //filter extraneous corrections that would be impossible
            if(Math.abs(ultrasonicCorrection)>3){
                ultrasonicCorrection = 0;
            }
            //Drive to target cryptobox column
            while (drive.moveIMU(drive.getEncoderDistance(), 16 * COUNTS_PER_INCH - (ultrasonicCorrection * COUNTS_PER_INCH), 0, 0, 14 * COUNTS_PER_INCH, .75,
                    0.75, -90, DEFAULT_PID_STRAFE, 0, DEFAULT_ERROR_DISTANCE, 1000) && opModeIsActive()) ;
            drive.resetEncoders();

            if(vumarkSeen.equals("RIGHT")){
                //pivot and deposit glyphs
                depositGlyphs(8*COUNTS_PER_INCH, 8*COUNTS_PER_INCH, 30, 300, true);
            }else if(vumarkSeen.equals("LEFT")){
                //pivot and deposit glyphs
                depositGlyphs(7*COUNTS_PER_INCH, 7*COUNTS_PER_INCH, -15, 300, true);
            }else{
                //pivot and deposit glyphs
                depositGlyphs(7*COUNTS_PER_INCH, 7*COUNTS_PER_INCH, 15, 300, true);
            }

            //Raise the lift to avoid hitting the glyphs
            lift.setTargetPosition(898);
            lift.setPower(1);
            drive.resetEncoders();

            //pivot to face glyph pit
            while (drive.pivotIMU(-145, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 250, Direction.FASTEST) && opModeIsActive());

            //Lower lift to position to intake glyphs
            lift.setTargetPosition(5);
            lift.setPower(1);
            drive.resetEncoders();

            //start up intake
            bottomIntake.secureGlyph();
            topIntake.secureGlyph();

            //Move into Glyph Pit 30 inches
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 33 * COUNTS_PER_INCH, 12 * COUNTS_PER_INCH, 0, 20 * COUNTS_PER_INCH, 0.7, 0.35, -145, DEFAULT_PID, -145, DEFAULT_ERROR_DISTANCE, 1000) && opModeIsActive()) ;

            //retrieve glyphs from the pit
            getGlyphsFarStone(-145, -145, 18);

            //back up from pit
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 33 * COUNTS_PER_INCH, 20*COUNTS_PER_INCH, 0, 9 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, .35, 35, DEFAULT_PID, -145, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive()) ;

            // We want to spin the glyph depending on what is in the intake.
            // Condition 1: Regular starting position and top glyph and bottom is not present, spin
            if(spin.getPosition() > .5 && topGlyphColor.getDistance(DistanceUnit.CM) <= 6 && !(bottomGlyphColor.getDistance(DistanceUnit.CM)<=6)){
                //Move to Spin Position
                spin.setPosition(SPIN_ROTATED);
                timer.reset();
                while(timer.milliseconds()<500&&opModeIsActive());

                // Condition 2: Flipped position and bottom (red) glyph and top is not present, spin.
            }else if(spin.getPosition() <.5 && bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6 && !(topGlyphColor.getDistance(DistanceUnit.CM)<=6)){
                //Move to Spin Position
                spin.setPosition(SPIN_START);
                timer.reset();
                while(timer.milliseconds()<500&&opModeIsActive());
            }
            if(vumarkSeen.equals("LEFT")){
                //Pivot to face cryptobox and go in at 190 degrees (less steep than previously) Go into the right box
                depositGlyphs(9*COUNTS_PER_INCH, 7*COUNTS_PER_INCH, 30, 550, false);

            }else if(vumarkSeen.equals("RIGHT")){
                //Pivot to face cryptobox
                depositGlyphs(6*COUNTS_PER_INCH, 5*COUNTS_PER_INCH, -15, 300, false);
                //while (drive.pivotIMU(195, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 250, Direction.FASTEST) && opModeIsActive()) ;
            }else{
                // Case where the Center or unknown. Pivot to face cryptobox an deposit into the right box
                depositGlyphs(6*COUNTS_PER_INCH, 5*COUNTS_PER_INCH, -15, 300, false);
            }
            drive.stop();
        }
        /*
         **************************************************************************************************************************************
         ***********************************************     BLUE STONE 2     *************************************************************
         ***************************************************************************************************************************************
         */

        else if(autoProgram.equals("BlueStone2")) {
            //Drive off of balancing stone
            while (drive.moveIMU(drive.getEncoderDistance(), 26 * COUNTS_PER_INCH, 0, 0, 14 * COUNTS_PER_INCH, .5,
                    DEFAULT_MIN_POWER, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive()) ;

            drive.stop();

            //initialize ultrasonic variables
            int i = 0;
            double ultrasonicSum=0;
            double ultrasonicReading;

            //get 5 readings of the ultrasonic sensor and sum them
            while(opModeIsActive()&&i<5){
                ultrasonicReading = ultrasonic_jewel.cmUltrasonic();
                timer.reset();
                while(ultrasonicReading==255&&opModeIsActive()&&timer.seconds()<1000){
                    ultrasonicReading=ultrasonic_jewel.cmUltrasonic();
                }
                ultrasonicSum+=ultrasonicReading;
                i++;
            }

            //average the ultrasonic values and determine how much more or less the robot needs to travel
            double ultrasonicAverage = ultrasonicSum/i;
            double ultrasonicCorrection = (ultrasonicAverage-36)/2.54;

            //filter on ultrasonic sensor in case bad values are returned
            if(Math.abs(ultrasonicCorrection)>3){
                ultrasonicCorrection = 0;
            }
            drive.resetEncoders();

            //Drive to the center position.
            while (drive.moveIMU(drive.getEncoderDistance(), 16 * COUNTS_PER_INCH - (ultrasonicCorrection * COUNTS_PER_INCH), 0, 0, 14 * COUNTS_PER_INCH, .75,
                    0.75, -90, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive()) ;

            drive.resetEncoders();

            if(vumarkSeen.equals("LEFT")){
                //pivot and deposit glyphs
                depositGlyphs(7*COUNTS_PER_INCH, 7*COUNTS_PER_INCH, 145, 300, true);
            }else if(vumarkSeen.equals("RIGHT")){
                //pivot and deposit glyphs
                depositGlyphs(4*COUNTS_PER_INCH, 4*COUNTS_PER_INCH, 195, 300, true);
            }else{
                //pivot and deposit glyphs
                depositGlyphs(4*COUNTS_PER_INCH, 4*COUNTS_PER_INCH, 165, 300, true);
            }

            //Raise the lift to prevent the glyph arms from hitting the glyph
            lift.setTargetPosition(898);
            lift.setPower(1);
            drive.resetEncoders();

            //pivot to face glyph pit
            while (drive.pivotIMU(-35, -90, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 2, 750, Direction.FASTEST) && opModeIsActive());
            lift.setTargetPosition(5);
            lift.setPower(1);
            drive.resetEncoders();

            //start up intake
            bottomIntake.secureGlyph();
            topIntake.secureGlyph();

            //Move into Glyph Pit 30 inches
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 33 * COUNTS_PER_INCH, 12 * COUNTS_PER_INCH, 0, 20 * COUNTS_PER_INCH, 0.7, 0.35, -35, DEFAULT_PID, -35, DEFAULT_ERROR_DISTANCE, 1000) && opModeIsActive()) ;

            //retrieve glyphs from the pit
            getGlyphsFarStone(-35, -35, 18);

            //back up from pit
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 33 * COUNTS_PER_INCH, 20*COUNTS_PER_INCH, 0, 9 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, .35, 145, DEFAULT_PID, -35, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive()) ;

            // We want to spin the glyph depending on what is in the intake.
            // Condition 1: Regular starting position and top glyph and bottom is not present, spin
            if(spin.getPosition() > .5 && topGlyphColor.getDistance(DistanceUnit.CM) <= 6 && !(bottomGlyphColor.getDistance(DistanceUnit.CM)<=6)){
                //Move to Spin Position
                spin.setPosition(SPIN_ROTATED);
                timer.reset();
                while(timer.milliseconds()<500&&opModeIsActive());

             // Condition 2: Flipped position and bottom (red) glyph and top is not present, spin.
            }else if(spin.getPosition() <.5 && bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6 && !(topGlyphColor.getDistance(DistanceUnit.CM)<=6)){
                //Move to Spin Position
                spin.setPosition(SPIN_START);
                timer.reset();
                while(timer.milliseconds()<500&&opModeIsActive());
            }
            /*
            while(opModeIsActive()){
                telemetry.addData("spin position", spin.getPosition());
                telemetry.addData("bottom glyph color", bottomGlyphColor.getDistance(DistanceUnit.CM));
                telemetry.addData("top glyph color", topGlyphColor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
*/

            if(vumarkSeen.equals("LEFT")){
                //Pivot to face cryptobox and go in at 190 degrees (less steep than previously) Go into the right box
                depositGlyphs(6*COUNTS_PER_INCH, 6*COUNTS_PER_INCH, 185, 300, false);

            }else if(vumarkSeen.equals("RIGHT")){
                //Pivot to face cryptobox
                depositGlyphs(10*COUNTS_PER_INCH, 7*COUNTS_PER_INCH, 140, 550, false);
                //while (drive.pivotIMU(195, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 250, Direction.FASTEST) && opModeIsActive()) ;
            }else{
                // Case where the Center or unknown. Pivot to face cryptobox an deposit into the right box
                depositGlyphs(6*COUNTS_PER_INCH, 6*COUNTS_PER_INCH, 185, 300, false);
            }
        }

        //Stop intake threads
        bottomIntake.stop();
        topIntake.stop();

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

        allTrackables.addAll(relicTrackables);
        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        OpenGLMatrix redTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        relicTemplate.setLocation(redTargetLocationOnField);
        RobotLog.ii(TAG, "Red Target=%s", format(redTargetLocationOnField));

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);


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
        relic_claw.setPosition(0);
        spin.setPosition(SPIN_START);
    }

    /**
     * Using the range sensors and the color sensor, the robot will automatically detect which stone the robot is on
     * and select the corresponding autonomous program
     */
    public void autoSelectAlignment(){
        OpenGLMatrix lastLocation = null;
        String zAngle = null;
        int zAngleNum = 0;
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
                    if(lastLocation != null){
                        if(zAngleNum == -180){
                            aligned = true;
                            led.turnOn();
                        }
                    }else {
                        aligned = true;
                        led.turnOn();
                    }

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

            for (VuforiaTrackable trackable : allTrackables) {
                /**
                 * getUpdatedRobotLocation() will return null if no new information is available since
                 * the last time that call was made, or if the trackable is not currently visible.
                 * getRobotLocation() will return null if the trackable is not currently visible.
                 */
                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(lastLocation));
                zAngle = format(lastLocation).substring(15).split(" ")[2];
                zAngleNum = Integer.parseInt(zAngle.substring(0, zAngle.length()-1)) - 20;
                telemetry.addData("Z Angle", Integer.parseInt(zAngle.substring(0, zAngle.length()-1)) - 20);
            } else {
                telemetry.addData("Pos", "Unknown");
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
        while(drive.pivotIMU(center+15, -90, .3, .25, 5, 0, Direction.FASTEST)&&opModeIsActive());
        while(drive.pivotIMU(center-15, -90, .3, .25, 5, 0, Direction.FASTEST)&&opModeIsActive());
        while(drive.pivotIMU(center, -90, .3, DEFAULT_MIN_POWER, 2, 500, Direction.FASTEST)&&opModeIsActive());
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

        /*//check if glyph in bottom arms of robot
        if (bottomGlyphColor.getDistance(DistanceUnit.CM)<=6){
            led.setLEDPower(1);
        }else{
            glyphWiggle(-90, 15);
            timer.reset();
            while(timer.milliseconds()<250&&opModeIsActive());
        }
        drive.resetEncoders();
        lift.setTargetPosition(300);*/

        getGlyphsFarStone(-90, -90, 18);

        //back up to cryptobox
        while(drive.moveIMU(drive.getEncoderDistance(), backupDistance, backUpAngle-20*COUNTS_PER_INCH, 0, backupDistance-10*COUNTS_PER_INCH, DEFAULT_MAX_POWER, backwardMin, backUpAngle, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

        if(spin.getPosition() == SPIN_START && topGlyphColor.getDistance(DistanceUnit.CM) <= 6 &&
                (bottomGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(bottomGlyphColor.getDistance(DistanceUnit.CM)))){
            spin.setPosition(SPIN_ROTATED);
            float spinPosition = (float) spin.getPosition();
            ReadWriteFile.writeFile(file, String.valueOf(spinPosition));
        }else if(spin.getPosition() == SPIN_ROTATED && bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6 &&
                (topGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(topGlyphColor.getDistance(DistanceUnit.CM)))){
            spin.setPosition(SPIN_START);
            float spinPosition = (float) spin.getPosition();
            ReadWriteFile.writeFile(file, String.valueOf(spinPosition));
        }

    }

    /**
     * Deposit glyphs in the robot's possesion into the glyph pit
     * @param depositDistance Distance the robot should travel to deposit the glyphs
     * @param comeBackDistance Distance the robot should travel after depositing the glyphs
     * @param depositAngle The angle the robot should move at when depositing a glyph in the pit
     * @param depositLiftPosition The position the lift should be in before depositing the glyph
     */
    public void depositGlyphs(double depositDistance, double comeBackDistance, double depositAngle, int depositLiftPosition, boolean lowerSecondTime){
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

        if(lowerSecondTime){
            //lower the lift
            lift.setTargetPosition(depositLiftPosition-200);
        }


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

    public void getGlyphsFarStone(double moveAngle, double orientation, double timeBeforeDeposit){


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

            //Raise lift to spin position
            lift.setTargetPosition(800);
            lift.setPower(1);

            //Back up 12 inches
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 12 * COUNTS_PER_INCH, 6*COUNTS_PER_INCH, 0, 6 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, moveAngle+180, DEFAULT_PID, orientation, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;

            //flip
            spin.setPosition(SPIN_ROTATED);
            timer.reset();
            while(timer.milliseconds() < 500 && opModeIsActive()); // waiting for spin to complete
            lift.setTargetPosition(5); // set target position to bottom
            lift.setPower(1);
            timer.reset();
            while(timer.milliseconds() < 250 && opModeIsActive()); // waiting for lift to be lowered

            //Drive 25 inches forward to pick up second glyph
            drive.resetEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
            while (drive.moveIMU(drive.getEncoderDistance(), 25 * COUNTS_PER_INCH, 20 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, .75,
                    0.3, moveAngle, DEFAULT_PID, orientation, 20, 750) && opModeIsActive());


            //check if glyph in bottom arms of robot (previously spun so the top is actually on bottom)
            if (topGlyphColor.getDistance(DistanceUnit.CM)<=6){
                led.setLEDPower(1);
            }else if (gameTime.seconds() < timeBeforeDeposit){
                glyphWiggle(orientation, 15);
                timer.reset();
            }

            //raise lift off ground
            lift.setTargetPosition(250);
            lift.setPower(1);

            //Back up 13 inches to original position
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 13 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0, 5 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, moveAngle+180, DEFAULT_PID, orientation, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
        }
        //CASE 2: THE ROBOT HAS 1 GLYPH IN THE TOP, BUT NOTHING IN THE BOTTOM
        else if((bottomGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(bottomGlyphColor.getDistance(DistanceUnit.CM)))&& topGlyphColor.getDistance(DistanceUnit.CM) <= 6){
            glyphCase = 2;

            drive.resetEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
            //Drive 10 inches forward to pick up second glyph
            while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0, 5 * COUNTS_PER_INCH, .5,
                    0.3, moveAngle, DEFAULT_PID, orientation, DEFAULT_ERROR_DISTANCE, 1000) && opModeIsActive());
            if(bottomGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(bottomGlyphColor.getDistance(DistanceUnit.CM))&&gameTime.seconds()<timeBeforeDeposit){
                glyphWiggle(moveAngle, 15);
            }

            //Raise lift to prevent glyph from dragging
            lift.setTargetPosition(250);
            lift.setPower(1);

            drive.resetEncoders();
            //Drive back 10 inches to return to original position
            while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0, 5 * COUNTS_PER_INCH, .5,
                    0.3, moveAngle+180, DEFAULT_PID, orientation, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive());

        }
        //CASE 3: THE ROBOT DOES NOT HAVE ANY GLYPHS
        else{
            glyphCase = 2;

            drive.resetEncoders(DcMotor.RunMode.RUN_USING_ENCODER);
            //Drive 10 inches forward to pick up second glyph
            while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0, 5 * COUNTS_PER_INCH, .5,
                    0.3, moveAngle, DEFAULT_PID, orientation, DEFAULT_ERROR_DISTANCE, 1000) && opModeIsActive());
            if((bottomGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(bottomGlyphColor.getDistance(DistanceUnit.CM))||topGlyphColor.getDistance(DistanceUnit.CM) > 6 || Double.isNaN(topGlyphColor.getDistance(DistanceUnit.CM)))&&gameTime.seconds()<timeBeforeDeposit){
                glyphWiggle(moveAngle, 15);
            }

            //Raise lift to prevent glyph from dragging
            lift.setTargetPosition(250);
            lift.setPower(1);

            drive.resetEncoders();
            //Drive back 10 inches to return to original position
            while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 10 * COUNTS_PER_INCH, 0, 5 * COUNTS_PER_INCH, .5,
                    0.3, moveAngle+180, DEFAULT_PID, orientation, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive());


        }
        //raise lift to spin position
        lift.setTargetPosition(800);
        lift.setPower(1);

    }

    /**
     * Reads ultrasonic sensor 5 times and averages the values. All undefined values (255) are thrown out and are not used in the average calculation
     * @param ultrasonic Ultrasonic sensor used to record the 5 sensor readings
     * @return distance from the target (based on the average of 5 values), in centimeters.
     */
    public double averageUltrasonic(ModernRoboticsI2cRangeSensor ultrasonic){
        //initialize ultrasonic variables
        int i = 0;
        double ultrasonicSum=0;
        double ultrasonicReading;

        //get 5 readings of the ultrasonic sensor and sum them
        timer.reset();
        while(opModeIsActive()&&i<5&&timer.milliseconds()<1000){
            ultrasonicReading = ultrasonic.cmUltrasonic();
            while((ultrasonicReading==255)&&opModeIsActive() && timer.milliseconds() < 1000){
                ultrasonicReading=ultrasonic.cmUltrasonic();
                telemetry.addData("Autonomous", "Reading Ultrasonic");
                telemetry.update();
            }
            ultrasonicSum+=ultrasonicReading;
            i++;
            telemetry.addData("Autonomous", "Reading Ultrasonic");
            telemetry.update();
        }

        //determine the average and correction due to ultrasonic sensor readings
        double ultrasonicAverage;
        if(i != 0){
            ultrasonicAverage = ultrasonicSum/i;
        }else{
            ultrasonicAverage = 0;
        }
        return ultrasonicAverage;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }
}