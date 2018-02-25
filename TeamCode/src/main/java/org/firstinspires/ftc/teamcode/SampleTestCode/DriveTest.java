package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Sarthak on 2/25/2018.
 */

public class DriveTest extends LinearOpMode {

    BNO055IMU boschIMU;
    IIMU imu;
    MecanumDriveTrain drive;
    DcMotor rf, rb, lf, lb;
    List<DcMotor> motors;

    @Override
    public void runOpMode() throws InterruptedException {
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motors = new ArrayList<>();
        motors.add(rf);
        motors.add(rb);
        motors.add(lf);
        motors.add(lb);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //Calibrate IMU
        telemetry.addData("Init", "IMU Calibrating");
        telemetry.update();
        boschIMU = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new BoschIMU(boschIMU);
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Init", "IMU Instantiated");
        telemetry.update();

        //initialize drivetrain
        drive = new MecanumDriveTrain(motors, imu, telemetry);
        drive.resetEncoders();
        telemetry.addData("Init", "Drivetrain and IMU Initialized");
        telemetry.update();

        waitForStart();

        drive.resetEncoders();
        double[] PID = {0.1};
        double encoder = drive.averageEncoders();
        while(drive.moveIMU(encoder, 1000, 750, 250, 0.75, 0.25,
                0, PID, 0, 1000) && opModeIsActive()){
            encoder = drive.averageEncoders();
            telemetry.addData("Current Position", encoder);
            telemetry.update();
        }

        while(opModeIsActive()){
            telemetry.addData("Current Position", drive.averageEncoders());
            telemetry.update();
        }

    }
}
