package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Enums.Direction;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Sarthak on 2/25/2018.
 */
@Autonomous(name = "Drive Test", group = "")
public class DriveTest extends LinearOpMode {

    BNO055IMU boschIMU;
    IIMU imu;
    MecanumDriveTrain drive;
    DcMotor rf, rb, lf, lb;
    List<DcMotor> motors;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardwareMap();
        setMotorBehaviors();

        initIMU();

        //initialize drivetrain
        drive = new MecanumDriveTrain(motors, imu, telemetry);
        drive.resetEncoders();
        telemetry.addData("Init", "Drivetrain and IMU Initialized");
        telemetry.update();

        waitForStart();

        drive.resetEncoders();
        double[] PID = {0.005};
        double encoder = drive.getEncoderDistance();
        while(drive.moveIMU(drive.getEncoderDistance(), 1000, 500, 0, .5, .2, 0, PID, 0, 20, 500)&&opModeIsActive()){

        }
        while(opModeIsActive()){
            telemetry.addData("Current Position", imu.getZAngle());
            telemetry.update();
        }

    }
    public void initHardwareMap(){
        //Hardware Map Motors
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");

        //Group drive motors in Array List
        motors = new ArrayList<>();
        motors.add(rf);
        motors.add(rb);
        motors.add(lf);
        motors.add(lb);

    }

    public void setMotorBehaviors(){
        //Set motor behaviors
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
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

}