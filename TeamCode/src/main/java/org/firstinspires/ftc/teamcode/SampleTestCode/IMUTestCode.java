package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;

/**
 * Created by Sarthak on 1/6/2018.
 */
@TeleOp(name = "IMU Discontinuity Test", group = "Test")
public class IMUTestCode extends LinearOpMode{
    IIMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        imu = new BoschIMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.initialize();
        imu.setOffset(0);
        telemetry.addData("Init", "IMU Instantiated");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("IMU", imu.getZAngle(90));
            telemetry.addData("Raw IMU", imu.getZAngle());
            telemetry.update();
        }
    }
}
