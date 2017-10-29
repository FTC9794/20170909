package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Jewel.TwoPointJewelArm;

/**
 * Created by Sarthak on 10/15/2017.
 */
@Autonomous(name = "TwoPointJewelArmTest", group = "Test")
public class TwoPointJewelArmTest extends LinearOpMode {

    LynxI2cColorRangeSensor lynx;
    Servo pan, tilt;

    TwoPointJewelArm jewel;

    IColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        lynx = (LynxI2cColorRangeSensor) hardwareMap.get("color");
        //pan = hardwareMap.servo.get("servoP");
        tilt = hardwareMap.servo.get("servoT");
        colorSensor = new LynxColorRangeSensor(lynx);
        jewel = new TwoPointJewelArm(pan, tilt, colorSensor, telemetry);

        //pan.setPosition(0.82);
        tilt.setPosition(0.5);

        waitForStart();
        while (opModeIsActive()){
            jewel.readColor();
            telemetry.update();
        }
    }
}
