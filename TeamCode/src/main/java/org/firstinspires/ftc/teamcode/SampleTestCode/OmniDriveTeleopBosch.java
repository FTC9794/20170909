package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.GamepadPlus;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.OmniDirectionalDrive;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.NavxIMU;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Sarthak on 10/29/2017.
 */
//@TeleOp(name = "Omni Drive Teleop Malificent", group = "Teleop")
public class OmniDriveTeleopBosch extends LinearOpMode {

    DcMotor rf, rb, lf, lb;

    IIMU imu;
    OmniDirectionalDrive drive;
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
        telemetry.addData("Init", "About to call IMU");
        telemetry.update();
        imu = new BoschIMU(hardwareMap.get(BNO055IMU.class, "imu"));
        telemetry.addData("Init"," About to set Zero");
        telemetry.update();
        imu.setAsZero();

        telemetry.addData("Init", "About to Init Drivetrain");
        //initialize drivetrain
        drive = new OmniDirectionalDrive(motors, imu, telemetry);
        drive.resetEncoders();
        gamepadPlus1 = new GamepadPlus(gamepad1);

        telemetry.addData("Init", "IMU and Drivetrain Instantiated");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("right front", rf.getPower());
            telemetry.addData("right back", rb.getPower());
            telemetry.addData("left front", lf.getPower());
            telemetry.addData("left back", lb.getPower());
            telemetry.addData("IMU", imu.getZAngle());
            telemetry.addData("Desired Angle", gamepadPlus1.getAngleLeftStick());
            telemetry.addData("Desired Power", gamepadPlus1.getDistanceFromCenterLeft());
            telemetry.update();
            drive.moveIMU(1, 0, gamepadPlus1.getDistanceFromCenterRight(), 1, gamepadPlus1.getAngleRightStick(), gamepadPlus1.getDistanceFromCenterLeft()*.02, 0, imu.getZAngle()+gamepadPlus1.leftStickX() * 40, false, 0);


        }
    }
}