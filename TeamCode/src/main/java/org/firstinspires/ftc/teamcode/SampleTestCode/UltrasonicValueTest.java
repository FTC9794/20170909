package org.firstinspires.ftc.teamcode.SampleTestCode;

import android.annotation.TargetApi;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DataLogger;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.IUltrasonic;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.MRRangeSensor;

import java.util.Date;

/**
 * Created by Sarthak on 12/31/2017.
 */
@TeleOp(name = "Ultrasonic Value and Filter Test", group = "test")
public class UltrasonicValueTest extends LinearOpMode{
    IUltrasonic ultrasonic;
    ModernRoboticsI2cRangeSensor us;

    @Override
    public void runOpMode() throws InterruptedException {
        us = (ModernRoboticsI2cRangeSensor) hardwareMap.get("jewel_us");
        ultrasonic = new MRRangeSensor(us);
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Raw US", us.cmUltrasonic());
            telemetry.addData("Filtered US Value", ultrasonic.cmDistance());
            telemetry.update();
        }
    }
}
