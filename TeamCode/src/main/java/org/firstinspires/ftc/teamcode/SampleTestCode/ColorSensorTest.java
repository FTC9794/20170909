package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;

/**
 * Created by Sarthak on 1/20/2018.
 */
@TeleOp(name = "Color Sensor Value Test", group = "Test")
public class ColorSensorTest extends LinearOpMode {
    IColorSensor color;
    LynxI2cColorRangeSensor floor_color;
    @Override
    public void runOpMode() throws InterruptedException {
        floor_color = (LynxI2cColorRangeSensor) hardwareMap.get("floor_color");
        color = new LynxColorRangeSensor(floor_color);
        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Floor Color Red", color.red());
            telemetry.update();
        }
    }
}
