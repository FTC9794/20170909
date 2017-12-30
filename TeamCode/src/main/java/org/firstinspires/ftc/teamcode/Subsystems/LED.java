package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sarthak on 12/30/2017.
 */

public class LED {

    private DcMotor led;
    private final double POWER_ON = 1;
    private final double POWER_OFF = 0;

    public LED(DcMotor led){
        this.led = led;
    }

    public boolean turnOn(){
        setLEDPower(POWER_ON);
        return true;
    }

    public boolean turnOff(){
        setLEDPower(POWER_OFF);
        return true;
    }

    public boolean setLEDPower(double power){
        this.led.setPower(power);
        return true;
    }

}
