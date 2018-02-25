package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.GamepadPlus;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AcceleratedDcMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.TankDrive;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;

import java.util.ArrayList;

/**
 * Created by Sarthak on 9/24/2017.
 */

@TeleOp(name = "Tank Drive Teleop Test", group = "Test")
@Disabled
public class TankDriveTeleop extends LinearOpMode {
    DcMotor rf, rb, lf, lb;
    IIMU imu;
    IDrivetrain drive;
    GamepadPlus gamepadPlus1, gamepadPlus2;
    ArrayList<DcMotor> motors;
    final double PIVOT_GAIN = 0.005;

    @Override
    public void runOpMode() throws InterruptedException {
        rf = new AcceleratedDcMotor(hardwareMap.dcMotor.get("right_front"), 0.1);
        rb = new AcceleratedDcMotor(hardwareMap.dcMotor.get("right_back"), 0.1);
        lf = new AcceleratedDcMotor(hardwareMap.dcMotor.get("left_front"), 0.1);
        lb = new AcceleratedDcMotor(hardwareMap.dcMotor.get("left_back"), 0.1);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        //create array list of motors
        motors = new ArrayList<>();
        motors.add(rf);
        motors.add(rb);
        motors.add(lf);
        motors.add(lb);

        //set motor modes and zero power behavior
        for(DcMotor motor: motors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //initialize IMU
        imu = new BoschIMU((BNO055IMU) hardwareMap.get("imu"));
        imu.calibrate();
        imu.setAsZero();

        //construct drivetrain
        drive = new TankDrive(motors, imu);

        gamepadPlus1 = new GamepadPlus(gamepad1);
        gamepadPlus2 = new GamepadPlus(gamepad2);

        //Add Telemetry data to let drivers know robot is ready to run
        telemetry.addData("Initialization", "complete");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            //drive.moveIMU(1, 0, gamepadPlus1.leftStickY(), 1, gamepadPlus1.getAngleRightStick(), PIVOT_GAIN*gamepadPlus1.getDistanceFromCenterRight(), 0, 0, false, 0);
        }
        drive.stop();
    }
}
