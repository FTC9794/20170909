package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.GandalfCode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.IGlyph;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.twoWheelIntake;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.IUltrasonic;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.RevRangeSensor;

import java.io.File;
import java.util.ArrayList;
import java.util.List;


/**
 * Created by ishaa on 1/28/2018.
 */

@TeleOp(name = "Lefty Teleop")
public class rewrittenLinearOpMode extends LinearOpMode {
    //hardware to be used on robot
    DcMotor lf, lb, rf, rb, lift, relic_extension;
    List<DcMotor> motors;
    CRServo rightWheel1, leftWheel1, rightWheel2, leftWheel2;
    Servo spin, pan, tilt, relic_claw, relic_arm, relic_tilt;
    BNO055IMU boschIMU;
    IIMU imu;
    LynxI2cColorRangeSensor glyphSensor1;
    LynxI2cColorRangeSensor glyphSensor2;
    DcMotor led_motor;
    LED leds;
    DigitalChannel glyphLimit, relicLimit;

    //timers to be used during teleop
    ElapsedTime intake1Time;
    ElapsedTime intake2Time;
    ElapsedTime rotateTime;

    //variables tracking different values of sensors, inputs and positions
    double pitch, roll, pivot;
    double xPowerBalance, yPowerBalance;
    double balanceAngle;
    int liftPosition;

    //enum for the glyph lift states
    private enum glyphLiftStates{
        MANUAL, POSITION
    }

    //enum for the rotation state
    private enum glyphRotateStates{
        LIFTING, ROTATING, LOWERING, STOPPED
    }

    //enum for the driving states
    private enum driveStates{
        MANUAL, MOVING_ON_STONE, BALANCING
    }

    //enum variables to store states
    driveStates driveState;
    glyphLiftStates liftState;
    glyphRotateStates rotateState;
    rewritten.intakeState lowerIntakeState;
    rewritten.intakeState upperIntakeState;

    //toggle variables
    boolean yPressed = false;
    boolean aPressed = false;
    boolean xPressed = false;
    boolean bPressed = false;

    //information of hardware variables
    boolean glyphIntakeRotated = false;
    boolean lowerLiftAfterRotating = false;
    boolean clawClosed = true;
    boolean intaking = false;
    boolean intakePressed = false;

    //constants for lift
    final double LIFT_POWER = 1;
    final double LIFT_HALF_POWER_DOWN = .1;
    final double LIFT_HALF_POWER_UP = .75;
    final int LIFT_UPPER_LIMIT = 2208;
    final int GLYPH_POSITION_0 = 0;
    final int GLYPH_POSITION_1 = 240;
    final int GLYPH_POSITION_2 = 912;
    final int GLYPH_POSITION_3 = 1584;
    final int GLYPH_POSITION_4 = 2208;
    final int LIFT_POSITION_OFFSET = 100;
    final int LIFT_INTAKEN_POSITION = 240;
    final int GLYPH_ROTATE_POSITION = 768;

    //constant for drivetrain movements and balancing
    final double [] pidGain = {0};
    final double DRIVE_LOW_SPEED = .5;
    final double DESIRED_Y_BALANCE_ANGLE = -.75;
    final double DESIRED_X_BALANCE_ANGLE = -2.25;
    final double BALANCE_GAIN = .015;

    //constant for gamepads
    final double ANALOG_PRESSED = .2;

    //constant for intake outake of glyphs
    final double INTAKE_POWER = .74;
    final double OUTAKE_POWER = -.74;
    final double GLYPH_GRAB_DISTANCE = 5.6;
    final double GLYPH_VISIBLE_TIME = 250;

    //constants for rotation
    final double SPIN_NORMAL_POSITION = .825;
    final double SPIN_SPUN_POSITION = 0;
    final double ROTATE_TIME = 750;

    //constants for jewel
    final double JEWEL_TILT_POSITION = 0.7;
    final double JEWEL_PAN_POSITION = .55;

    //constants for relic
    final double RELIC_ARM_ORIGIN = .01;
    final double RELIC_ARM_GRAB_POS = .865;
    final double RELIC_ARM_EXTENSION_HALF_POWER = .5;
    final double RELIC_ARM_RETRACTION_HALF_POWER = -.5;
    final double RELIC_ARM_EXTENSION_FULL_POWER = 1;
    final double RELIC_ARM_RETRACTION_FULL_POWER = -1;

    //objects for different subsystems
    IGlyph bottomIntake, topIntake;
    IUltrasonic glyphColor1;
    IUltrasonic glyphColor2;
    ClawThreePoint relic;
    MecanumDriveTrain drive;

    //pid values for balancing
    double pGainx = .055;
    double iGainx = .0000;
    double dGainx = 4;
    double pGainy = .055;
    double iGainy = .0000;
    double dGainy = 4;
    double xDesired = 0;
    double yDesired = 0;
    double currentTime = 0;
    double previousTime = 0;
    double currentDifferencex = 0;
    double previousDifferencex = 0;
    double currentDifferencey = 0;
    double previousDifferencey = 0;
    double areaSumx = 0;
    double areaSumy = 0;
    double correctionx = 0;
    double correctiony = 0;
    double currentValuex = 0;
    double currentValuey = 0;
    double slopex;
    double slopey;
    double px;
    double py;
    double ix;
    double iy;
    double dx;
    double dy;
    ElapsedTime PIDTimer;

    LynxVoltageSensor voltageSensor;


    /*
     **************************************************************************************************************************************
     **********************************************     PROGRAM STARTS HERE     *************************************************************
     ***************************************************************************************************************************************
     */
    @Override
    public void runOpMode() throws InterruptedException {


        //initialize robot hardware
        initHardwareMap();
        setMotorBehaviors();
        initIMU();

        //Get spin position from autonomous
        String fileName = "autoSpinPosition.txt";
        File file = AppUtil.getInstance().getSettingsFile(fileName);
        String spinString = ReadWriteFile.readFile(file);
        double spinInitPosition = Double.parseDouble(spinString);

        //create color sensor objects
        glyphColor1 = new RevRangeSensor(glyphSensor1);
        glyphColor2 = new RevRangeSensor(glyphSensor2);

        //set State of different subsystems
        liftState = glyphLiftStates.POSITION;
        lowerIntakeState = rewritten.intakeState.NOTHING;
        upperIntakeState = rewritten.intakeState.NOTHING;
        rotateState = glyphRotateStates.STOPPED;
        driveState = driveStates.MANUAL;

        //create timers
        intake1Time = new ElapsedTime();
        intake2Time = new ElapsedTime();
        rotateTime = new ElapsedTime();
        PIDTimer = new ElapsedTime();

        //create intakes
        bottomIntake = new twoWheelIntake(leftWheel1, rightWheel1, INTAKE_POWER, OUTAKE_POWER);
        topIntake = new twoWheelIntake(leftWheel2, rightWheel2, INTAKE_POWER, OUTAKE_POWER);

        //create relic mechanism
        relic = new ClawThreePoint(relic_extension, relic_arm, relic_tilt, relic_claw);

        //create drivetrain
        drive = new MecanumDriveTrain(motors, imu, telemetry);

        xDesired = imu.getXAngle();
        yDesired = imu.getYAngle();
        telemetry.addData("Initialization", "complete");
        telemetry.update();

        // WAIT FOR START
        waitForStart();

        /*
         **************************************************************************************************************************************
         ***********************************************     OPMODE RUNS HERE     *************************************************************
         ***************************************************************************************************************************************
         */
        //set servo initialization positions once play has been pressed
        pan.setPosition(JEWEL_PAN_POSITION);
        tilt.setPosition(JEWEL_TILT_POSITION);
        relic.pickUpRelic();
        relic.setTiltPosition(1);
        relic_arm.setPosition(0);

        if(spinInitPosition == 0){ //If the glyph spun in autonomous, initialize it to the rotated position here
            glyphIntakeRotated = true;
        }else{ //If the glyph did not spin in autonomous, initialize it to the original initialization position
            glyphIntakeRotated = false;
        }


        //while stop hasn't been pressed and the program is running
        while(opModeIsActive()) {

            drivetrainStateMachine();
            liftStateMachine();


            //toggle to determine whether the robot is intaking or no
            if (gamepad1.left_bumper) {
                if (!intakePressed) {
                    intaking = !intaking;
                }
                intakePressed = true;
            } else {
                intakePressed = false;
            }

            bottomIntakeStateMachine();
            topIntakeStateMachine();
            rotateStateMachine();
            controlLEDS();
            relicControls();

            telemetry.addData("Relic Tilt Position", relic_tilt.getPosition());
            telemetry.addData("Relic Arm Position", relic_arm.getPosition());
            telemetry.update();

            float spinPosition = (float) spin.getPosition();
            ReadWriteFile.writeFile(file, String.valueOf(spinPosition));

        }


    }
    public void initHardwareMap(){


        //Hardware Map Motors
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        lift = hardwareMap.dcMotor.get("glyph_lift");
        relic_extension = hardwareMap.dcMotor.get("relic_extension");
        led_motor = hardwareMap.dcMotor.get("leds");
        leds = new LED(led_motor);


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
        glyphSensor1 = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor1");
        glyphSensor2 = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor2");

        //Hardware Map Limit Switches
        glyphLimit = hardwareMap.digitalChannel.get("glyph_limit");
        relicLimit = hardwareMap.digitalChannel.get("relic_limit");


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
    public void setMotorBehaviors(){
        //add motors to a list
        motors = new ArrayList<>();
        motors.add(rf);
        motors.add(rb);
        motors.add(lf);
        motors.add(lb);

        //reset motor encoders
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relic_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set motor behaviors
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
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

    }
    public void liftStateMachine(){
        //state machine to lift glyph lift to certain positions
        switch(liftState) {

            //state for manually controlling the lift
            case MANUAL:

                //set position of the lift variable to current position of the lift
                liftPosition = lift.getCurrentPosition();
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                //if neither of the triggers are pressed move into the position state
                if (gamepad2.left_trigger < ANALOG_PRESSED && gamepad2.right_trigger < ANALOG_PRESSED) {
                    liftState = glyphLiftStates.POSITION;

                    //Check if right trigger is pressed (to go up) and if the position of the lift is less than the upper limit
                } else if (gamepad2.right_trigger >= ANALOG_PRESSED && liftPosition < LIFT_UPPER_LIMIT) {

                    //move lift up at power depending on whether the half speed control is pressed
                    if(gamepad2.right_bumper){
                        lift.setPower(LIFT_HALF_POWER_UP);
                    }else{
                        lift.setPower(LIFT_POWER);
                    }

                    //Check if the left trigger is pressed (to go down) and if the limit switch isn't pressed
                } else if (gamepad2.left_trigger >= ANALOG_PRESSED && glyphLimit.getState()) {

                    //move the lift down at power depending on whether the half speed control is pressed
                    if(gamepad2.right_bumper){
                        lift.setPower(-LIFT_HALF_POWER_DOWN);
                    }else{
                        lift.setPower(-LIFT_POWER);
                    }

                    //if the limit switch is pressed reset the encoders and set the current position variable to 0
                } else if (!glyphLimit.getState()) {
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftPosition = 0;
                    lift.setPower(0);

                    //if the a button is pressed increment the lift down to the next lowest position and change into position mode
                } else if (gamepad2.a) {

                    //determine next lift position based on current lift position
                    if (liftPosition < GLYPH_POSITION_1 + LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_0;
                    } else if (liftPosition < GLYPH_POSITION_2 + LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_1;
                    } else if (liftPosition < GLYPH_POSITION_3 + LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_2;
                    } else {
                        liftPosition = GLYPH_POSITION_3;
                    }

                    //set toggle variable to true
                    aPressed = true;

                    //change the state of the lift
                    liftState = glyphLiftStates.POSITION;

                    //if the y button is pressed increment the lift up to the next highest position and change into position mode
                } else if (gamepad2.y) {

                    //determine next position based on current position
                    if (liftPosition > GLYPH_POSITION_3 - LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_4;
                    } else if (liftPosition > GLYPH_POSITION_2 - LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_3;
                    } else if (liftPosition > GLYPH_POSITION_1 - LIFT_POSITION_OFFSET) {
                        liftPosition = GLYPH_POSITION_2;
                    } else {
                        liftPosition = GLYPH_POSITION_1;
                    }

                    //set toggle variable
                    yPressed = true;

                    //change the state of the lift
                    liftState = glyphLiftStates.POSITION;
                }
                break;
            //if the lift is the position state
            case POSITION:

                //if the lift is supposed to go to the bottom or to a position of 0 do the following
                if (liftPosition <= 0) {

                    //if the position is below 0, set it to 0
                    liftPosition = 0;

                    //check if the limit switch is pressed
                    if (glyphLimit.getState()) {

                        //if the limit switch is not pressed, lower the lift
                        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        lift.setPower(-LIFT_POWER);
                    } else {

                        //if the limit switch is pressed, stop the lift and reset the encoder
                        lift.setPower(0);
                        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }

                    //if the lift isn't supposed to go to the bottom
                } else {

                    //set the motor modes and move the lift to the desired encoder position
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(liftPosition);
                    lift.setPower(LIFT_POWER);
                }

                //switch into the manual state if the triggers are pressed
                if (gamepad2.left_trigger > ANALOG_PRESSED || gamepad2.right_trigger > ANALOG_PRESSED) {
                    liftState = glyphLiftStates.MANUAL;
                    lift.setPower(0);

                    //increment down to next level lower if a is pressed
                } else if (gamepad2.a) {

                    //determine if toggle condition holds
                    if (!aPressed) {

                        //determine what the next level down is based on current position
                        if (liftPosition < GLYPH_POSITION_1 + LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_0;
                        } else if (liftPosition < GLYPH_POSITION_2 + LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_1;
                        } else if (liftPosition < GLYPH_POSITION_3 + LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_2;
                        } else {
                            liftPosition = GLYPH_POSITION_3;
                        }

                        //set toggle variable to true
                        aPressed = true;

                    }

                    //if y is pressed increment lift up to next highest position
                } else if (gamepad2.y) {

                    //determine if toggle case is true
                    if (!yPressed) {

                        //determine which position to move to depending on current position
                        if (liftPosition > GLYPH_POSITION_3 - LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_4;
                        } else if (liftPosition > GLYPH_POSITION_2 - LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_3;
                        } else if (liftPosition > GLYPH_POSITION_1 - LIFT_POSITION_OFFSET) {
                            liftPosition = GLYPH_POSITION_2;
                        } else {
                            liftPosition = GLYPH_POSITION_1;
                        }

                        //set toggle variable to true
                        yPressed = true;
                    }

                }

                //set toggle variable to false if button is released.
                if(!gamepad2.y){
                    yPressed = false;
                }
                if(!gamepad2.a){
                    aPressed = false;
                }
                break;
        }//end of lift state machine
    }
    public void drivetrainStateMachine(){
        //Drivetrain state machine
        switch(driveState){

            //if drivetrain is in manual mode
            case MANUAL:

                //store gamepad values as variables
                pitch = -gamepad1.left_stick_y;
                roll = gamepad1.left_stick_x;
                pivot = gamepad1.right_stick_x;

                //set to low power if trigger pressed
                if(gamepad1.right_trigger>.2||gamepad1.left_stick_button||gamepad1.right_stick_button){
                    //multiply the powers by the low speed powers
                    rf.setPower((pitch-roll-pivot)*DRIVE_LOW_SPEED);
                    rb.setPower((pitch+roll-pivot)*DRIVE_LOW_SPEED);
                    lf.setPower((pitch+roll+pivot)*DRIVE_LOW_SPEED);
                    lb.setPower((pitch-roll+pivot)*DRIVE_LOW_SPEED);
                }else{
                    //set to regular powers if the trigger isn't pressed
                    rf.setPower(pitch-roll-pivot);
                    rb.setPower(pitch+roll-pivot);
                    lf.setPower(pitch+roll+pivot);
                    lb.setPower(pitch-roll+pivot);
                }
                //determine if need to move into a different state
                if(gamepad1.b&&!bPressed){
                    //change the state of the drivetrain
                    driveState = driveStates.MOVING_ON_STONE;

                    //set the angle at which to climb the stone to the current angle
                    balanceAngle = imu.getZAngle();

                    //reset encoders
                    drive.resetEncoders();

                }

                //determine if needed to correct just balancing
                if(gamepad1.y){

                    //set the state to balancing
                    driveState = driveStates.BALANCING;

                    resetBalancingVariables();
                }
                break;
            //state for moving onto the stone using encoders
            case MOVING_ON_STONE:

                //drive onto the balancing stone using the drivetrain method
                if(!drive.moveIMU(drive.getEncoderDistance(), 1000, 0, 0, 990, .75, .3, balanceAngle+180, pidGain, balanceAngle, 20, 250)){
                    //once method is complete change states to balancing
                    driveState = driveStates.BALANCING;
                    resetBalancingVariables();
                }

                //go to manual state if joysticks moved or b pressed
                if((gamepad1.b&&!bPressed)||gamepad1.left_stick_x!=0||gamepad1.left_stick_y!=0||gamepad1.right_stick_x!=0){
                    drive.resetEncoders();
                    driveState = driveStates.MANUAL;
                }
                break;

            //state for balancing on stone after all 4 wheels are on stone
            case BALANCING:
                currentValuex = imu.getXAngle();
                currentValuey = imu.getYAngle();
                currentTime = PIDTimer.milliseconds();
                currentDifferencex = currentValuex - xDesired;
                currentDifferencey = currentValuey - yDesired;
                px = currentDifferencex * pGainx;
                py = currentDifferencey * pGainy;

                areaSumx += ((previousDifferencex+currentDifferencex)/2)*(currentTime-previousTime);
                areaSumy += ((previousDifferencey+currentDifferencey)/2)*(currentTime-previousTime);

                ix = areaSumx*iGainx;
                iy = areaSumy*iGainy;

                slopex = (previousDifferencex-currentDifferencex)/(previousTime-currentTime);
                slopey = (previousDifferencey-currentDifferencey)/(previousTime-currentTime);

                dx = slopex*dGainx;
                dy = slopey*dGainy;

                correctionx = px+ix+dx;
                correctiony = py+iy+dy;

                rf.setPower(correctiony-correctionx);
                rb.setPower(correctiony+correctionx);
                lf.setPower(correctiony+correctionx);
                lb.setPower(correctiony-correctionx);

                previousTime = currentTime;
                previousDifferencex = currentDifferencex;
                previousDifferencey = currentDifferencey;
                //leave this state when b pressed or joysticks moved
                if((gamepad1.b&&!bPressed)||gamepad1.left_stick_x!=0||gamepad1.left_stick_y!=0||gamepad1.right_stick_x!=0){
                    drive.resetEncoders();
                    driveState = driveStates.MANUAL;
                }
        }//end of drivetrain state machine

        //set variable for toggle to whether b is pressed or not
        bPressed = gamepad1.b;
    }
    public void bottomIntakeStateMachine(){
        //state machine for lower intake
        switch(lowerIntakeState){

            //nothing state motors are off, just checking whether to move to another state
            case NOTHING:

                //Check if the right bumper or if the up and down eject buttons are pressed and the spin is the right orientation to go to the outake state
                if(gamepad1.right_bumper||(!glyphIntakeRotated&&gamepad1.dpad_down)||(glyphIntakeRotated&&gamepad1.dpad_up)){
                    lowerIntakeState = rewritten.intakeState.OUTAKE;

                    //If the robot is intaking, move to the intake state
                }else if (intaking) {
                    lowerIntakeState = rewritten.intakeState.INTAKE_MOTOR;
                    intake1Time.reset();

                    //Turn off the motors if not changing states
                }else{
                    bottomIntake.turnOff();
                }
                break;

            //intake with motors on state
            case INTAKE_MOTOR:

                //if the color sensor sees a glyph and the timer has gone past the time the sensor needs to see the glyph then move into the intake no motor state
                if(glyphColor1.cmDistance() < GLYPH_GRAB_DISTANCE&&intake1Time.milliseconds()>GLYPH_VISIBLE_TIME){
                    lowerIntakeState = rewritten.intakeState.INTAKE_NO_MOTOR;

                    //if moving to the intake no motor state and the bottom intake is on the bottom then lift the lift to get glyphs off the ground
                    if(!glyphIntakeRotated&&lift.getCurrentPosition()<LIFT_INTAKEN_POSITION){
                        liftPosition = LIFT_INTAKEN_POSITION;
                    }

                    //otherwise if the right bumper is pressed or the up or down dpad based on how the mechanism is spun, move to the outake state and set intaking to false
                }else if(gamepad1.right_bumper||(!glyphIntakeRotated&&gamepad1.dpad_down)||(glyphIntakeRotated&&gamepad1.dpad_up)){
                    lowerIntakeState = rewritten.intakeState.OUTAKE;
                    intaking = false;

                    //if the robot is not intaking set the state to nothing to turn off everything
                }else if(!intaking){
                    lowerIntakeState = rewritten.intakeState.NOTHING;

                    //if the robot does not see a glyph, reset the timer and turn on the motors
                }else if(glyphColor1.cmDistance() > GLYPH_GRAB_DISTANCE){
                    intake1Time.reset();
                    bottomIntake.secureGlyph();

                    //this only happens when the robot has seen a glyph and if that is the case we do not reset the timer so that we know how long we have seen the glyph for
                }else{
                    bottomIntake.secureGlyph();
                }
                break;

            //this is the case for the state where the robot is intaking, but the motors are off since there is already a glyph in our robot
            case INTAKE_NO_MOTOR:

                //if the robot does not see a glyph then switch to the intake motor state to suck the glyph back into our robot
                if(glyphColor1.cmDistance()>GLYPH_GRAB_DISTANCE){
                    lowerIntakeState = rewritten.intakeState.INTAKE_MOTOR;
                    intake1Time.reset();

                    //if the right bumper is pressed or the dpad up or down depending on the intake orientation, the robot will stop intaking and this intake will move to the outake state
                }else if(gamepad1.right_bumper||(!glyphIntakeRotated&&gamepad1.dpad_down)||(glyphIntakeRotated&&gamepad1.dpad_up)){
                    lowerIntakeState = rewritten.intakeState.OUTAKE;
                    intaking = false;

                    //if the robot is no longer intaking, this intake will move to the nothing state to turn off motors
                }else if(!intaking){
                    lowerIntakeState = rewritten.intakeState.NOTHING;

                    //Otherwise, the robot will turn off the motors to stop the wheels from spinning when we have a glyph
                }else{
                    bottomIntake.turnOff();
                }
                break;

            //This is the outake state where the robot is ejecting the glyph
            case OUTAKE:

                //if one of the outake buttons is not held down, then the robot will move into the nothing state
                if(!gamepad1.right_bumper&&!(!glyphIntakeRotated&&gamepad1.dpad_down)&&!(glyphIntakeRotated&&gamepad1.dpad_up)){
                    lowerIntakeState = rewritten.intakeState.NOTHING;

                    //otherwise, the robot will outake a glyph
                }else{
                    bottomIntake.dispenseGlyph();
                }
                break;
        }//end of bottom intake state machine
    }
    public void topIntakeStateMachine(){
        //state machine for lower intake
        switch(upperIntakeState){

            //nothing state motors are off, just checking whether to move to another state
            case NOTHING:

                //Check if the right bumper or if the up and down eject buttons are pressed and the spin is the right orientation to go to the outake state
                if(gamepad1.right_bumper||(glyphIntakeRotated&&gamepad1.dpad_down)||(!glyphIntakeRotated&&gamepad1.dpad_up)){
                    upperIntakeState = rewritten.intakeState.OUTAKE;

                    //If the robot is intaking, move to the intake state
                }else if (intaking) {
                    upperIntakeState = rewritten.intakeState.INTAKE_MOTOR;
                    intake2Time.reset();

                    //Turn off the motors if not changing states
                }else{
                    topIntake.turnOff();
                }
                break;

            //intake motor state for top intake state machine
            case INTAKE_MOTOR:

                //if there is a glyph in this intake and it has seen that glyph for more milliseconds than the glyph visible time then change to the intake no motor state
                if(glyphColor2.cmDistance() < GLYPH_GRAB_DISTANCE&&intake2Time.milliseconds()>GLYPH_VISIBLE_TIME){
                    upperIntakeState= rewritten.intakeState.INTAKE_NO_MOTOR;

                    //lift the lift barely off the ground if this intake is the bottom intake to prevent the glyph from falling out
                    if(glyphIntakeRotated&&lift.getCurrentPosition()<LIFT_INTAKEN_POSITION){
                        liftPosition = LIFT_INTAKEN_POSITION;
                    }

                    //If the outake buttons are pressed, then move to the outake state and set the robot as not intaking
                }else if(gamepad1.right_bumper||(glyphIntakeRotated&&gamepad1.dpad_down)||(!glyphIntakeRotated&&gamepad1.dpad_up)){
                    upperIntakeState = rewritten.intakeState.OUTAKE;
                    intaking = false;

                    //If the robot is no longer intaking then move to the nothing state
                }else if(!intaking){
                    upperIntakeState = rewritten.intakeState.NOTHING;

                    //if the robot does not have a glyph, then reset the timer and turn on the motors
                }else if(glyphColor2.cmDistance() > GLYPH_GRAB_DISTANCE){
                    intake2Time.reset();
                    topIntake.secureGlyph();

                    //if the robot has a glyph then don't reset the timer to determine how long it has had it for and keep running the motors
                }else{
                    topIntake.secureGlyph();
                }
                break;

            //intake no motor state is used when a glyph is in the robot, the wheels are off, but the robot is still intaking
            case INTAKE_NO_MOTOR:

                //if a glyph is no longer in the intake, go to the intake motor state to turn the wheels back on
                if(glyphColor2.cmDistance()>GLYPH_GRAB_DISTANCE){
                    upperIntakeState = rewritten.intakeState.INTAKE_MOTOR;
                    intake2Time.reset();

                    //if an eject button is pressed, move into the outake state
                }else if(gamepad1.right_bumper||(glyphIntakeRotated&&gamepad1.dpad_down)||(!glyphIntakeRotated&&gamepad1.dpad_up)){
                    upperIntakeState = rewritten.intakeState.OUTAKE;
                    intaking = false;

                    //if the robot is no longer intaking, move into the nothing state
                }else if(!intaking){
                    upperIntakeState = rewritten.intakeState.NOTHING;

                    //otherwise turn off the motor and wait for an event to happen so that the wheels and glyph inside the robot do not wear out
                }else{
                    topIntake.turnOff();
                }
                break;

            //outake state for ejecting glyphs
            case OUTAKE:

                //determine if all of the outake buttons are not pressed, if they are all not pressed, move to the nothing state
                if(!gamepad1.right_bumper&&!(glyphIntakeRotated&&gamepad1.dpad_down)&&!(!glyphIntakeRotated&&gamepad1.dpad_up)){
                    upperIntakeState = rewritten.intakeState.NOTHING;

                    //if any of them are pressed, outake
                }else{
                    topIntake.dispenseGlyph();
                }
                break;
        }//end of bottom intake state machine

    }


    public void rotateStateMachine(){
        //rotate state machine is used to automatically lift our glyph lift to a safe height, rotate our intake mechanism and lower the lift
        switch(rotateState){

            //in the stop state, nothing is movine
            case STOPPED:

                //check whether the intake is rotated or not and set the servo position based on that
                if(glyphIntakeRotated){
                    spin.setPosition(SPIN_SPUN_POSITION);
                }else{
                    spin.setPosition(SPIN_NORMAL_POSITION);
                }

                //if the left bumper is pressed and the glyph lift isn't being manually controlled then begin the rotating proceedure
                if(gamepad2.left_bumper&&liftState!= glyphLiftStates.MANUAL){

                    //if the glyph lift is below the safe height for rotating, move into the lifting state and set variable to lower lift after finished rotating
                    if(liftPosition < GLYPH_ROTATE_POSITION){
                        rotateState = glyphRotateStates.LIFTING;
                        liftPosition = GLYPH_ROTATE_POSITION+LIFT_POSITION_OFFSET;
                        lowerLiftAfterRotating = true;

                        //if the glyph lift is above the safe height for rotating, move into the rotating state without lifting and don't lower lift after done rotating
                    }else{
                        rotateState = glyphRotateStates.ROTATING;
                        glyphIntakeRotated = !glyphIntakeRotated;
                        rotateTime.reset();
                        lowerLiftAfterRotating = false;
                    }
                }
                break;

            //lifting state used to lift glyph lift to safe position to rotate
            case LIFTING:

                //if the driver takes control of the lift, stop the rotating procedure
                if(liftState == glyphLiftStates.MANUAL){
                    rotateState = glyphRotateStates.STOPPED;
                }

                //if the lift is above the rotating position, move into the rotating state and set the variable appropriately
                if(lift.getCurrentPosition()>=GLYPH_ROTATE_POSITION){
                    rotateState = glyphRotateStates.ROTATING;
                    glyphIntakeRotated = !glyphIntakeRotated;
                    rotateTime.reset();
                }
                break;

            //rotating state used for rotating intake
            case ROTATING:

                //determine if it has been enough time for the intake to rotate
                if(rotateTime.milliseconds()>ROTATE_TIME){

                    //if the lift needs to lower after rotating, go to the lowering state
                    if(lowerLiftAfterRotating){
                        rotateState = glyphRotateStates.LOWERING;
                        liftPosition = 0;

                        //if the lift does not need to lower after rotating, go to the stopped state
                    }else{
                        rotateState = glyphRotateStates.STOPPED;
                    }

                    //if the driver takes control of the glyph lift, stop rotating procedure
                }else if(liftState == glyphLiftStates.MANUAL){
                    rotateState = glyphRotateStates.STOPPED;

                    //move the servo to the desired position based on the variable set when leaving previous state to come to rotating state
                }else if(glyphIntakeRotated){
                    spin.setPosition(SPIN_SPUN_POSITION);
                }else{
                    spin.setPosition(SPIN_NORMAL_POSITION);
                }
                break;

            //lowering state is to bring lift to ground after it has finished rotating
            case LOWERING:

                //if the lift has reached the desired position or the driver has taken control of the glyph lift then the robot will go into the stopped state
                if(lift.getCurrentPosition()<LIFT_POSITION_OFFSET||liftState == glyphLiftStates.MANUAL){
                    rotateState = glyphRotateStates.STOPPED;
                }
        }
    }


    public void controlLEDS(){

        //determine which intake is on the bottom
        if(glyphIntakeRotated){

            //if upper intake on bottom and upper intake has a glyph then turn on lights
            if(upperIntakeState== rewritten.intakeState.INTAKE_NO_MOTOR){
                leds.setLEDPower(.5);
            }else{
                leds.setLEDPower(0);
            }
        }else{

            //if bottom intake on bottom and bottom intake has a glyph then turn on lights
            if(lowerIntakeState== rewritten.intakeState.INTAKE_NO_MOTOR){
                leds.setLEDPower(.5);
            }else{
                leds.setLEDPower(0);
            }
        }
    }

    public void relicControls(){
        //Relic Extension Motor Controls with Encoder Limits
        if (gamepad2.right_bumper) {
            relic.extend(RELIC_ARM_EXTENSION_FULL_POWER, gamepad2.dpad_up && relic_extension.getCurrentPosition() < 2112);
        } else{
            relic.extend(RELIC_ARM_EXTENSION_HALF_POWER, gamepad2.dpad_up && relic_extension.getCurrentPosition() < 2112);
        }

        //relic retraction controls with encoder limits
        if(gamepad2.right_bumper){
            if(relic_extension.getCurrentPosition() > 250){
                relic.retract(RELIC_ARM_RETRACTION_FULL_POWER, gamepad2.dpad_down && relicLimit.getState());
            }else{
                relic.retract(RELIC_ARM_RETRACTION_HALF_POWER, gamepad2.dpad_down && relicLimit.getState());
            }
        }else{
            relic.retract(RELIC_ARM_RETRACTION_HALF_POWER, gamepad2.dpad_down && relicLimit.getState());
        }

        //if nothing is being pressed, don't extend or retract
        if(!gamepad2.dpad_up && !gamepad2.dpad_down){
            relic.extensionPowerZero();
            if(!relicLimit.getState()){
                relic.resetRelicEncoder();
            }
        }

        //check if the button is pressed and it wasn't pressed in the last loop cycle
        if(gamepad2.x&&!xPressed){

            //check to see if the claw was opened or closed
            if(clawClosed){
                //open the claw
                relic.releaseRelic();
            }else{
                //close the claw
                relic.pickUpRelic();
            }
            //set variables based on updated status of robot
            clawClosed=!clawClosed;
            xPressed = true;
        }else if(!gamepad2.x){//check if the gamepad wasn't pressed
            //if it wasn't pressed set the variable accordingly
            xPressed = false;
        }

        //set relic to preset positions based on different buttons
        if(gamepad2.dpad_left){
            relic.setArmPosition(RELIC_ARM_GRAB_POS);
        }else if(gamepad1.a){
            relic.setArmPosition(RELIC_ARM_ORIGIN);
            relic.setTiltPosition(1);
        }
        if(gamepad2.b){
            relic.setTiltPosition(0.6);
            relic.setArmPosition(0.6);
        }

        //Relic Arm Servo Controls to keep relic always standing regardless of the angle the arm is at
        if (relic.returnArmPos()< .4) {
            relic.adjustArm((-gamepad2.right_stick_y > 0.1 && relic.returnArmPos() <= 1), 0.05);
            relic.adjustArm((-gamepad2.right_stick_y < -0.1 && relic.returnArmPos() >= 0.04), -0.05);

        } else {
            relic.adjustArm(-gamepad2.right_stick_y > 0.1 && relic.returnArmPos() <= 1, .005);
            relic.adjustArm(-gamepad2.right_stick_y < -0.1 && relic.returnArmPos() >= 0.04, -.005);
        }

        //Relic Tilt Servo Controls to adjust tilt servo position
        relic.tiltRelic(-gamepad2.left_stick_y > 0.1 && relic.returnTiltPos() <= 0.9, 0.01);
        relic.tiltRelic(-gamepad2.left_stick_y < -0.1 && relic.returnTiltPos() >= 0.01, -0.01);
    }

    public void resetBalancingVariables(){
        currentTime = 0;
        previousTime = 0;
        areaSumx = 0;
        areaSumy = 0;
        correctionx = 0;
        correctiony = 0;
        currentValuex = imu.getXAngle();
        currentValuey = imu.getYAngle();
        pGainx = (-.005/2.1)*getBatteryVoltage()+.083;
        pGainy = pGainx;
        telemetry.addData("pgain", pGainx);
        telemetry.addData("battery", getBatteryVoltage());
        telemetry.update();
        PIDTimer.reset();
        currentDifferencex = currentValuex-xDesired;
        previousDifferencex = currentDifferencex;
        currentDifferencey = currentValuey-yDesired;
        previousDifferencey = currentDifferencey;


    }
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

}
