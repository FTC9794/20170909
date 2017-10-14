package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.OmniDirectionalDrive;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.NavxIMU;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Sarthak on 10/6/2017.
 */

@Autonomous(name = "Omni Directional Auto Demo", group = "Test")
public class OmniDirectionalAutonomousDemo extends LinearOpMode {
    DcMotor rf, rb, lf, lb;
    AHRS navx;

    IIMU imu;
    OmniDirectionalDrive drive;
    List<DcMotor> motors;
    final double COUNTS_PER_INCH = 45;
    final double ENCODER_OFFSET = 30;
    final double HIGH_MAX_POWER = 1;
    final double MID_MAX_POWER = 1;
    final double MIN_POWER = .5;
    final double PIVOT_GAIN = .008;
    final double HIGH_O_GAIN = .04;
    final double LOW_O_GAIN = .025;
    final double QUICK_PIVOT_TIME = 500;
    final double LONG_PIVOT_TIME = 750;
    @Override
    public void runOpMode() throws InterruptedException {
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

        //Add Telemetry data to let drivers know robot is ready to run
        telemetry.addData("Initialization", "complete");
        telemetry.update();
        waitForStart();
/*
        while(opModeIsActive()&&drive.move(0.5, 0.2, 0, 0, 0, 0, 0.0001, 90, true, 1000));
*/
        while(opModeIsActive()&&drive.move(.5, MIN_POWER, 12*COUNTS_PER_INCH-drive.averageEncoders(), 0.003, 45, .025, .003, 0,
                24*COUNTS_PER_INCH - drive.averageEncoders() < ENCODER_OFFSET && 24*COUNTS_PER_INCH - drive.averageEncoders() > -ENCODER_OFFSET, 1000));

/*
        while(opModeIsActive()&&drive.move(.5, MIN_POWER, 12*COUNTS_PER_INCH-drive.averageEncoders(), .003, 0, .025, .01, 90, 24*COUNTS_PER_INCH-drive.averageEncoders()<ENCODER_OFFSET&&24*COUNTS_PER_INCH-drive.averageEncoders()>-ENCODER_OFFSET, 500)){
            telemetry.addData("power", 24*COUNTS_PER_INCH-drive.averageEncoders()*.003);
        }
        drive.resetEncoders();

        while(opModeIsActive() && drive.move(.5, .2, 0, 0, 0, 0, .001, 90, true, .1)){
            telemetry.addData("Program", "Pivoting");
            telemetry.update();
        }
        /*
        while(opModeIsActive()){
            drive.slideAngle(0, 0, 1, .1, (-24)*COUNTS_PER_INCH+drive.averageEncoders(), .002, .05);
            telemetry.addData("difference", (-24)*COUNTS_PER_INCH-drive.averageEncoders());
            telemetry.addData("encoder average", drive.averageEncoders());
            telemetry.update();

        }*/

        /**
         *
         * @param highPower the maximum power for the robot to move
         * @param lowPower the lowest power for the robot to move
         * @param powerChange however close the robot is to the final target
         * @param powerGain the gain for how much to slow down as the robot approaches the final target
         * @param moveAngle the angle at which the robot will move in the frame of reference of the field
         * @param oGain the gain for the robot to correct the orientation
         * @param pGain the gain at which the robot will correct the orientation when pivoting
         * @param endOrientationAngle the angle at which the robot will end up at and follow while moving
         * @param endCondition whether the robot has reached the desired position
         * @param timeAfterAngle how much the robot should pivot after being +- 5 degrees of the endOrientationAngle
         */
/*
        while(opModeIsActive()&&drive.move(.5, MIN_POWER, 12*COUNTS_PER_INCH-drive.averageEncoders(), .003, 0, .025, .01, 90, 24*COUNTS_PER_INCH-drive.averageEncoders()<ENCODER_OFFSET&&24*COUNTS_PER_INCH-drive.averageEncoders()>-ENCODER_OFFSET, 500)){
            telemetry.addData("power", 24*COUNTS_PER_INCH-drive.averageEncoders()*.003);
        }
        drive.resetEncoders();


        while(opModeIsActive()/*&&drive.move(MID_MAX_POWER, MIN_POWER, 24*COUNTS_PER_INCH-drive.averageEncoders(), .003, -45, .025, .01, 45, (24)*COUNTS_PER_INCH-drive.averageEncoders()<5&&(24)*COUNTS_PER_INCH-drive.averageEncoders()>-5, 500)*) {
         /*   drive.slideAngle(-45, 45, .65, .25, 24 * COUNTS_PER_INCH - drive.averageEncoders(), .003, .01);
        }*/
        drive.stop();
    }
}
