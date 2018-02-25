package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GamepadPlus;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.NavxIMU;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Sarthak on 10/29/2017.
 */
//@TeleOp(name = "Omni Drive Teleop", group = "Teleop")
public class OmniDriveTeleop extends LinearOpMode {

    DcMotor rf, rb, lf, lb;
    AHRS navx;

    IIMU imu;
    MecanumDriveTrain drive;
    List<DcMotor> motors;

    GamepadPlus gamepadPlus1;

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
        navx = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("Device Interface Module 1"),
                0,
                AHRS.DeviceDataType.kProcessedData);
        imu = new NavxIMU(navx);
        imu.calibrate();
        imu.setAsZero();

        //initialize drivetrain
        drive = new MecanumDriveTrain(motors, imu, telemetry);
        drive.resetEncoders();
        gamepadPlus1 = new GamepadPlus(gamepad1);

        telemetry.addData("Init", "IMU and Drivetrain Instantiated");
        telemetry.update();
        waitForStart();
        imu.setOffset(45);
        while (opModeIsActive()) {
            telemetry.addData("right front", rf.getPower());
            telemetry.addData("right back", rb.getPower());
            telemetry.addData("left front", lf.getPower());
            telemetry.addData("left back", lb.getPower());
            telemetry.addData("IMU", imu.getZAngle());
            telemetry.addData("Desired Angle", gamepadPlus1.getAngleLeftStick());
            telemetry.addData("Desired Power", gamepadPlus1.getDistanceFromCenterLeft());
            telemetry.update();
            //drive.moveIMU(1, 0, gamepadPlus1.getDistanceFromCenterRight(), 1, gamepadPlus1.getAngleRightStick(), gamepadPlus1.getDistanceFromCenterLeft()*.02, 0, imu.getZAngle()+gamepadPlus1.leftStickX() * 40, false, 0);


        }
    }
}