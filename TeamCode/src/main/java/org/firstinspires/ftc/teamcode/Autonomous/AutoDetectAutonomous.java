package org.firstinspires.ftc.teamcode.Autonomous;

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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Enums.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.Enums.Alliance.RED;
import static org.firstinspires.ftc.teamcode.Enums.Alliance.UNKNOWN;

/**
 * Created by Sarthak on 2/7/2018.
 */
@Autonomous(name = "Auto Select Autonomous", group = "Autonomous")
public class AutoDetectAutonomous extends LinearOpMode {
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
            while(!jewel.knockOffJewel(RED, true, true)){
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
            //check vumark and move to the target column
            if(vumarkSeen.equals("LEFT")){
                while(drive.moveIMU(drive.getEncoderDistance(), CENTER_STONE_1_DIST+COLUMN_OFFSET, 0, 0, CENTER_STONE_1_DIST+COLUMN_OFFSET - 20*COUNTS_PER_INCH, .5, 0.25, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

            }else if(vumarkSeen.equals("RIGHT")){
                while(drive.moveIMU(drive.getEncoderDistance(), CENTER_STONE_1_DIST-COLUMN_OFFSET, 0, 0, CENTER_STONE_1_DIST-COLUMN_OFFSET-20*COUNTS_PER_INCH, .5, 0.25, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

            }else{
                while(drive.moveIMU(drive.getEncoderDistance(), CENTER_STONE_1_DIST, 0, 0, CENTER_STONE_1_DIST-20*COUNTS_PER_INCH, .5, .25, 0, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500)&&opModeIsActive());

            }
            //Deposit preloaded glyph
            depositGlyphs(5*COUNTS_PER_INCH, 8*COUNTS_PER_INCH, 75, 500);


            //Pivot to face Glyph Pit
            while(drive.pivotIMU(-90, -30, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 1, 500, Direction.FASTEST)&&opModeIsActive());

            //strafe to otherside of column if right
            if(vumarkSeen.equals("RIGHT")){
                while(drive.moveIMU(drive.getEncoderDistance(), 8*COUNTS_PER_INCH, 0, 0, 0, DEFAULT_MAX_POWER, .75, 0, DEFAULT_PID, -90, 10, 500)&&opModeIsActive());
            }

            //get glyphs from glyph pit
            getGlyphs(28*COUNTS_PER_INCH, 28*COUNTS_PER_INCH, 90, .75, DEFAULT_MIN_POWER);


            drive.resetEncoders();

            //Check if there is a glyph in the robot to determine whether to deposit a glyph or not
            if(bottomGlyphColor.getDistance(DistanceUnit.CM)<=6||topGlyphColor.getDistance(DistanceUnit.CM)<=6){
                //Deposit glyphs
                if(vumarkSeen.equals("RIGHT")){
                    depositGlyphs(8*COUNTS_PER_INCH, 8*COUNTS_PER_INCH, 105, 898);
                }else{
                    depositGlyphs(8*COUNTS_PER_INCH, 8*COUNTS_PER_INCH, 75, 898);
                }

                //pivot to face the glyph pit
                while(drive.pivotIMU(-90, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 2, 500, Direction.FASTEST)&&opModeIsActive());

            }

            drive.resetEncoders();

            //Align to the next column to deposit glyphs
            if(vumarkSeen.equals("LEFT")){
                while(drive.moveIMU(drive.getEncoderDistance(), COLUMN_OFFSET, COLUMN_OFFSET/2, 0, COLUMN_OFFSET, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 180, DEFAULT_PID, -90, 10, 500)&&opModeIsActive());

            }else if(!vumarkSeen.equals("RIGHT")){
                while(drive.moveIMU(drive.getEncoderDistance(), COLUMN_OFFSET+5.7*COUNTS_PER_INCH, COLUMN_OFFSET/2, 0, COLUMN_OFFSET, DEFAULT_MAX_POWER, .65, 0, DEFAULT_PID, -90, 10, 500)&&opModeIsActive());

            }
            drive.stop();
/*
**************************************************************************************************************************************
***********************************************     BLUE STONE 1     *************************************************************
***************************************************************************************************************************************
 */
        }else if(autoProgram.equals("BlueStone1")){
            //check vumark and drive to the target cryptobox column
            if(vumarkSeen.equals("LEFT")){
                while(drive.moveIMU(drive.getEncoderDistance(), CENTER_STONE_1_DIST-COLUMN_OFFSET + BLUE_ALLIANCE_OFFSET, 0, 0, 18*COUNTS_PER_INCH+COLUMN_OFFSET, .5, DEFAULT_MIN_POWER, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250)&&opModeIsActive());

            }else if(vumarkSeen.equals("RIGHT")){
                while(drive.moveIMU(drive.getEncoderDistance(), CENTER_STONE_1_DIST+COLUMN_OFFSET+BLUE_ALLIANCE_OFFSET, 0, 0, 18*COUNTS_PER_INCH-COLUMN_OFFSET, .5, DEFAULT_MIN_POWER, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250)&&opModeIsActive());

            }else{
                while(drive.moveIMU(drive.getEncoderDistance(), CENTER_STONE_1_DIST+BLUE_ALLIANCE_OFFSET, 0, 0, 18*COUNTS_PER_INCH, .5, DEFAULT_MIN_POWER, 180, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250)&&opModeIsActive());

            }

            //Deposit preloaded glyph
            depositGlyphs(7*COUNTS_PER_INCH, 8*COUNTS_PER_INCH, 105, 500);

            //Pivot to face glyph pit
            while(drive.pivotIMU(-90, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 500, Direction.FASTEST)&&opModeIsActive());

            if(vumarkSeen.equals("LEFT")){
                drive.resetEncoders();
                while(drive.moveIMU(drive.getEncoderDistance(), 8*COUNTS_PER_INCH, 0, 0, 0, .75, .75, 180, DEFAULT_PID, -90, DEFAULT_ERROR_DISTANCE, 250));
            }

            //Get glyphs from pit
            getGlyphs(28*COUNTS_PER_INCH, 28*COUNTS_PER_INCH, 90, .75, DEFAULT_MIN_POWER);

            //Check if there is a glyph in the robot to determine whether to deposit a glyph or not
            if(bottomGlyphColor.getDistance(DistanceUnit.CM)<=6||topGlyphColor.getDistance(DistanceUnit.CM)<=6) {
                //Deposit glyphs from first trip to glyph pit
                if (vumarkSeen.equals("LEFT")) {
                    depositGlyphs(8 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 75, 898);
                } else {
                    depositGlyphs(8 * COUNTS_PER_INCH, 8 * COUNTS_PER_INCH, 105, 898);
                }
                //Pivot to face the glyph pit
                while(drive.pivotIMU(-90, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 2, 500, Direction.FASTEST)&&opModeIsActive());
            }

            drive.resetEncoders();

            //Align to a different cryptobox column
            if(vumarkSeen.equals("RIGHT")){
                while(drive.moveIMU(drive.getEncoderDistance(), COLUMN_OFFSET, COLUMN_OFFSET/2, 0, COLUMN_OFFSET, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 0, DEFAULT_PID, -90, 10, 500)&&opModeIsActive());

            }else if(!vumarkSeen.equals("LEFT")){
                while(drive.moveIMU(drive.getEncoderDistance(), COLUMN_OFFSET+10*COUNTS_PER_INCH, COLUMN_OFFSET/2, 0, COLUMN_OFFSET, DEFAULT_MAX_POWER, .4, 180, DEFAULT_PID, -90, 10, 500)&&opModeIsActive());

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
            while (drive.moveIMU(drive.getEncoderDistance(), 25 * COUNTS_PER_INCH, 0, 0, 14 * COUNTS_PER_INCH, .5,
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
            double ultrasonicAverage = ultrasonicSum/5;
            double ultrasonicCorrection = (ultrasonicAverage-36)/2.54;

            //filter extraneous corrections that would be impossible
            if(Math.abs(ultrasonicCorrection)>3){
                ultrasonicCorrection = 0;
            }

            //Drive to target cryptobox column
            if (vumarkSeen.equals("RIGHT")) {
                while (drive.moveIMU(drive.getEncoderDistance(), 17 * COUNTS_PER_INCH - COLUMN_OFFSET - ultrasonicCorrection, 0, 0, 14 * COUNTS_PER_INCH, .75,
                        0.75, -90, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 1000) && opModeIsActive());
            } else if (vumarkSeen.equals("LEFT")) {
                while (drive.moveIMU(drive.getEncoderDistance(), 17 * COUNTS_PER_INCH + COLUMN_OFFSET - ultrasonicCorrection, 0, 0, 14 * COUNTS_PER_INCH, .75,
                        0.75, -90, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 1000) && opModeIsActive());
            } else
                while (drive.moveIMU(drive.getEncoderDistance(), 17 * COUNTS_PER_INCH - ultrasonicCorrection, 0, 0, 14 * COUNTS_PER_INCH, .75,
                        0.75, -90, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 1000) && opModeIsActive());
            drive.resetEncoders();

            //Pivot to face cryptobox
            while (drive.pivotIMU(15, 0, .4, DEFAULT_MIN_POWER_PIVOT, 2, 250, Direction.FASTEST) && opModeIsActive()) ;

            //Move lift into target position to deposit glyphs
            lift.setTargetPosition(300);
            lift.setPower(1);

            //drive into cryptobox to deposit the glyphs
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 5 * COUNTS_PER_INCH, 0, 0, 2 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 15, DEFAULT_PID, 15, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) {
                telemetry.addData("ultrasonic", ultrasonic_front_top.cmUltrasonic());
                telemetry.update();
            }
            //deposit glyph
            bottomIntake.dispenseGlyph();
            timer.reset();
            while (timer.milliseconds() < 250 && opModeIsActive()) ;

            //Back away from cryptobox
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 8 * COUNTS_PER_INCH, 4 * COUNTS_PER_INCH, 0, 2 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, -165, DEFAULT_PID, 15, DEFAULT_ERROR_DISTANCE, 200) && opModeIsActive()) ;
            drive.resetEncoders();

            //Raise the lift to avoid hitting the glyphs
            lift.setTargetPosition(898);
            lift.setPower(1);
            drive.resetEncoders();

            //Strafe towards middle of the field
            if (vumarkSeen.equals("RIGHT")) {
                while(drive.pivotIMU(0, 15, .5, .4, 2, 250, Direction.FASTEST));
                while (drive.moveIMU(drive.getEncoderDistance(), COLUMN_OFFSET * 2, 0, 0, 14 * COUNTS_PER_INCH, .75,
                        0.6, -90, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive()) ;
            } else if (vumarkSeen.equals("CENTER") || vumarkSeen.equals("")) {
                while(drive.pivotIMU(0, 15, .5, .4, 2, 250, Direction.FASTEST));

                while (drive.moveIMU(drive.getEncoderDistance(), COLUMN_OFFSET, 0, 0, 14 * COUNTS_PER_INCH, .75,
                        0.6, -90, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive()) ;
            }
            drive.resetEncoders();

            //pivot to face glyph pit
            while (drive.pivotIMU(-155, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 250, Direction.FASTEST) && opModeIsActive());

            //Lower lift to position to intake glyphs
            lift.setTargetPosition(5);
            lift.setPower(1);
            drive.resetEncoders();

            //start up intake
            bottomIntake.secureGlyph();
            topIntake.secureGlyph();

            //move into glyph pit
            while (drive.moveIMU(drive.getEncoderDistance(), 40 * COUNTS_PER_INCH, 17.5 * COUNTS_PER_INCH, 0, 26 * COUNTS_PER_INCH, 1,
                    0.75, -155, DEFAULT_PID, -155, DEFAULT_ERROR_DISTANCE, 250) && opModeIsActive()) ;

            //Wait for glyphs to come into the robot
            timer.reset();
            while (opModeIsActive() && timer.milliseconds() < 250) {

            }

            //check if glyph in bottom arms of robot
            if (bottomGlyphColor.getDistance(DistanceUnit.CM)<=6){
                led.setLEDPower(1);
            }else{
                glyphWiggle(-155, 15);
                timer.reset();
                while(timer.milliseconds()<250&&opModeIsActive());
            }
            drive.resetEncoders();

            //Raise lift to raise the glyphs once they are in the robot
            lift.setTargetPosition(898);

            //back up from pit
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 40 * COUNTS_PER_INCH, 25*COUNTS_PER_INCH, 0, 9 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 25, DEFAULT_PID, -155, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
            drive.resetEncoders();

            //Pivot to face cryptobox
            while (drive.pivotIMU(15, 105, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 250, Direction.FASTEST) && opModeIsActive()) ;
            drive.resetEncoders();

            //Deposit glyphs into column
            if(!vumarkSeen.equals("LEFT")){
                lift.setTargetPosition(200);
                lift.setPower(1);
            }else{
                lift.setTargetPosition(898);
                lift.setPower(1);
            }
            //drive into cryptobox
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 6 * COUNTS_PER_INCH, 6 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 15, DEFAULT_PID, 15, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) {
                telemetry.addData("ultrasonic", ultrasonic_front_top.cmUltrasonic());
                telemetry.update();
            }

            //deposit glyph
            bottomIntake.dispenseGlyph();
            topIntake.dispenseGlyph();
            timer.reset();
            while (timer.milliseconds() < 250 && opModeIsActive()) ;

            //Back away from cryptobox
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 9 * COUNTS_PER_INCH, 4 * COUNTS_PER_INCH, 0, 2 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, -165, DEFAULT_PID, 15, DEFAULT_ERROR_DISTANCE, 200) && opModeIsActive()) ;

            //Reset the drive encoders and set the drive power to zero
            drive.resetEncoders();
            drive.stop();

            //Turn off LEDs
            led.turnOff();

            //Strafe towards relic away from cryptobox
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 8 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 2 * COUNTS_PER_INCH, .8, .65, 105, DEFAULT_PID, 15, DEFAULT_ERROR_DISTANCE, 200) && opModeIsActive()) ;
            drive.stop();

            //Pivot to face glyph pit in preparation for teleop
            while (drive.pivotIMU(-135, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 250, Direction.FASTEST) && opModeIsActive()) ;
            drive.stop();
            drive.resetEncoders();

            //back into safe zone
            while (drive.moveIMU(drive.getEncoderDistance(), 5 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 2 * COUNTS_PER_INCH, .8, .65, 45, DEFAULT_PID, -135, DEFAULT_ERROR_DISTANCE, 200) && opModeIsActive()) ;
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

            //Drive to target cryptobox column using ultrasonic distance and preprogramed distance
            if (vumarkSeen.equals("LEFT")) {
                while (drive.moveIMU(drive.getEncoderDistance(), 17 * COUNTS_PER_INCH - COLUMN_OFFSET - ultrasonicCorrection, 0, 0, 14 * COUNTS_PER_INCH, .75,
                        0.75, -90, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
            } else if (vumarkSeen.equals("RIGHT")) {
                while (drive.moveIMU(drive.getEncoderDistance(), 17 * COUNTS_PER_INCH + COLUMN_OFFSET - ultrasonicCorrection, 0, 0, 14 * COUNTS_PER_INCH, .75,
                        0.75, -90, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
            } else
                while (drive.moveIMU(drive.getEncoderDistance(), 17 * COUNTS_PER_INCH - ultrasonicCorrection, 0, 0, 14 * COUNTS_PER_INCH, .75,
                        0.75, -90, DEFAULT_PID, 0, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
            drive.resetEncoders();

            //Pivot to face cryptobox
            while (drive.pivotIMU(165, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 250, Direction.FASTEST) && opModeIsActive()) ;

            //Deposit glyphs into cryptobox column
            lift.setTargetPosition(300);
            lift.setPower(1);

            //drive into cryptobox
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 4 * COUNTS_PER_INCH, 0, 0, 2 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 165, DEFAULT_PID, 165, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) {
                telemetry.addData("ultrasonic", ultrasonic_front_top.cmUltrasonic());
                telemetry.update();
            }
            //deposit glyph
            bottomIntake.dispenseGlyph();
            timer.reset();
            while (timer.milliseconds() < 250 && opModeIsActive()) ;
            //Back away from cryptobox

            timer.reset();
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 7 * COUNTS_PER_INCH, 4 * COUNTS_PER_INCH, 0, 2 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, -15, DEFAULT_PID, 165, DEFAULT_ERROR_DISTANCE, 200)&&timer.milliseconds()<1000 && opModeIsActive()) ;
            drive.resetEncoders();

            //Raise the lift to prevent the glyph arms from hitting the glyph
            lift.setTargetPosition(898);
            lift.setPower(1);

            //Strafe towards middle of the field
            if (vumarkSeen.equals("LEFT")) {
                while (drive.pivotIMU(180, 0, DEFAULT_MAX_POWER, .35, 2, 250, Direction.FASTEST) && opModeIsActive())
                    ;
                while (drive.moveIMU(drive.getEncoderDistance(), COLUMN_OFFSET * 2, 0, 0, 14 * COUNTS_PER_INCH, .75,
                        0.75, -90, DEFAULT_PID, 180, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;
            } else if (vumarkSeen.equals("CENTER") || vumarkSeen.equals("")) {
                while (drive.pivotIMU(180, 0, DEFAULT_MAX_POWER, .35, 2, 250, Direction.FASTEST) && opModeIsActive())
                    ;
                while (drive.moveIMU(drive.getEncoderDistance(), COLUMN_OFFSET, 0, 0, 14 * COUNTS_PER_INCH, .75,
                        0.75, -90, DEFAULT_PID, 180, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive())
                    ;
            }

            drive.resetEncoders();

            //pivot to face glyph pit
            while (drive.pivotIMU(-15, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 250, Direction.FASTEST) && opModeIsActive())
                ;
            lift.setTargetPosition(5);
            lift.setPower(1);
            drive.resetEncoders();

            //start up intake
            bottomIntake.secureGlyph();
            topIntake.secureGlyph();

            //Move into Glyph Pit
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 40 * COUNTS_PER_INCH, 40 * COUNTS_PER_INCH, 0, 6 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MAX_POWER, -15, DEFAULT_PID, -15, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;

            //Wait for glyphs to come into the robot
            timer.reset();
            while (opModeIsActive() && timer.milliseconds() < 250) {

            }

            //check if glyph in bottom arms of robot
            if (bottomGlyphColor.getDistance(DistanceUnit.CM) <= 6) {
                led.setLEDPower(1);
            } else {
                glyphWiggle(-15, 15);
                timer.reset();
                while (timer.milliseconds() < 250 && opModeIsActive()) ;
            }
            drive.resetEncoders();

            //lift glyphs out of pit
            lift.setTargetPosition(898);

            //back up from pit
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 40 * COUNTS_PER_INCH, 0, 0, 9 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 165, DEFAULT_PID, -15, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) ;

            //Pivot to face cryptobox
            while (drive.pivotIMU(165, 0, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER_PIVOT, 2, 250, Direction.FASTEST) && opModeIsActive()) ;

            drive.resetEncoders();

            //move glyph lift to correct height to deposit
            if(!vumarkSeen.equals("RIGHT")){
                lift.setTargetPosition(200);
                lift.setPower(1);
            }else{
                lift.setTargetPosition(898);
                lift.setPower(1);
            }

            //drive into cryptobox
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 10 * COUNTS_PER_INCH, 6 * COUNTS_PER_INCH, 0, 10 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, 165, DEFAULT_PID, 165, DEFAULT_ERROR_DISTANCE, 500) && opModeIsActive()) {
                telemetry.addData("ultrasonic", ultrasonic_front_top.cmUltrasonic());
                telemetry.update();
            }

            //deposit glyph
            bottomIntake.dispenseGlyph();
            topIntake.dispenseGlyph();
            timer.reset();
            while (timer.milliseconds() < 250 && opModeIsActive()) ;

            //Back away from cryptobox
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 9 * COUNTS_PER_INCH, 4 * COUNTS_PER_INCH, 0, 2 * COUNTS_PER_INCH, DEFAULT_MAX_POWER, DEFAULT_MIN_POWER, -15, DEFAULT_PID, 165, DEFAULT_ERROR_DISTANCE, 200) && opModeIsActive()) ;

            //Strafe towards relic away from cryptobox
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 8 * COUNTS_PER_INCH, 5 * COUNTS_PER_INCH, 0, 2 * COUNTS_PER_INCH, .8, .65, 75, DEFAULT_PID, 165, DEFAULT_ERROR_DISTANCE, 200) && opModeIsActive()) ;
            drive.stop();

            //Pivot to face glyph pit in preparation for teleop
            while (drive.pivotIMU(-45, 0, 0.4, DEFAULT_MIN_POWER_PIVOT, 2, 250, Direction.FASTEST) && opModeIsActive()) ;

            //move backward to get into safe zone
            drive.resetEncoders();
            while (drive.moveIMU(drive.getEncoderDistance(), 4 * COUNTS_PER_INCH, 2 * COUNTS_PER_INCH, 0, 2 * COUNTS_PER_INCH, .8, .65, 135, DEFAULT_PID, -45, DEFAULT_ERROR_DISTANCE, 200) && opModeIsActive()) ;
            drive.stop();
            led.turnOn();

        }

        //Stop intake threads
        bottomIntake.stop();
        topIntake.stop();

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