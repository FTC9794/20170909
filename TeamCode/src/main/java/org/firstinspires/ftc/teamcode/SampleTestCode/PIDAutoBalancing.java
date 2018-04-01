package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.GlyphIntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.IGlyph;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.LED;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.IUltrasonic;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ishaa on 4/1/2018.
 */

@Autonomous(name="PID Balance")
public class PIDAutoBalancing extends LinearOpMode {

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
    GlyphIntakeState lowerIntakeState;
    GlyphIntakeState upperIntakeState;

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
    final double RELIC_ARM_GRAB_POS = .78;
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

    //pid values
    final double pGainx = .055;
    final double iGainx = 0;
    final double dGainx = 6;
    final double pGainy = .055;
    final double iGainy = 0;
    final double dGainy = 6;
    final double xDesired = -2.25;
    final double yDesired = -.75;
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


    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        initIMU();
        setMotorBehaviors();
        PIDTimer = new ElapsedTime();
        waitForStart();
        PIDTimer.reset();
        while(opModeIsActive()){
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

            telemetry.update();
            correctionx = px+ix+dx;
            correctiony = py+iy+dy;

            rf.setPower(correctiony-correctionx);
            rb.setPower(correctiony+correctionx);
            lf.setPower(correctiony+correctionx);
            lb.setPower(correctiony-correctionx);

            previousTime = currentTime;
            previousDifferencex = currentDifferencex;
            previousDifferencey = currentDifferencey;
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
}
