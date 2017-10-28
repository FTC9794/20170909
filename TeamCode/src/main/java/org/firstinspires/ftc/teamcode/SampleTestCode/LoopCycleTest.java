package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DataLogger;

import java.util.Date;

/**
 * Created by Sarthak on 9/28/2017.
 */
@TeleOp(name = "Loop Cycle Test", group = "Test")
@Disabled
public class LoopCycleTest extends LinearOpMode {
    //Declare motors and sensors
    DcMotor rf, rb, lf, lb;
    //ModernRoboticsI2cRangeSensor ultrasonic;
    //Data logging
    DataLogger data;
    Date day = new Date();

    @Override
    public void runOpMode() throws InterruptedException {
        //Map motors
        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        //Map sensors
        //ultrasonic = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasonic");
        //Set up data logging
        data = new DataLogger(day.toString() + " Loop Cycle Test");
        data.addField("RF Encoder Value"); data.addField("RB Encoder Value");
        data.addField("LF Encoder Value"); data.addField("LB Encoder Value");
        //data.addField("Ultrasonic Value");
        data.newLine();
        //Wait for start
        waitForStart();

        while(opModeIsActive()){
            //Log motor encoder values
            data.addField(rf.getCurrentPosition()); data.addField(rb.getCurrentPosition());
            data.addField(rb.getCurrentPosition()); data.addField(lb.getCurrentPosition());
            //Log sensor value
            //data.addField((float) ultrasonic.cmUltrasonic());
            //New line
            data.newLine();
        }
    }
}
