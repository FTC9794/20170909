package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ishaa on 4/9/2018.
 */

@TeleOp(name = "RUN LEDS")
@Disabled
public class LEDDemo extends OpMode {
    DcMotor LED;
    @Override
    public void init() {
        LED = hardwareMap.dcMotor.get("leds");
    }

    @Override
    public void loop() {
        LED.setPower(1);
    }
}
