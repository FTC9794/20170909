package org.firstinspires.ftc.teamcode.Demos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ishaa on 4/9/2018.
 */

@TeleOp(name = "LED Demo")
@Disabled
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
        if(!increasing){
            ledPower -= 0.003;
        }else{
            ledPower += 0.003;
        }

        if(ledPower >= 1){
            increasing = false;
        }
        if(ledPower <= 0){
            increasing = true;
        }

        LED.setPower(ledPower);
        telemetry.addData("LED Power", ledPower);
        telemetry.addData("Led < 0?", ledPower <= 0);
        telemetry.addData("Increasing?", increasing);
        telemetry.update();
    }
}
