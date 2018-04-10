package org.firstinspires.ftc.teamcode.Demos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ishaa on 4/9/2018.
 */

@TeleOp(name = "LED Demo")
public class LEDDemo extends OpMode {
    DcMotor LED;
    double ledPower = 1;
    boolean increasing = false;
    @Override
    public void init() {
        LED = hardwareMap.dcMotor.get("leds");
    }

    @Override
    public void loop() {
        if(ledPower > 0 || !increasing){
            ledPower = ledPower - 0.002;
            increasing = false;
        }else if (ledPower <= 0 || increasing){
            ledPower = ledPower + 0.002;
            increasing = true;
        }
        LED.setPower(ledPower);
        telemetry.addData("LED Power", ledPower);
        telemetry.update();
    }
}
