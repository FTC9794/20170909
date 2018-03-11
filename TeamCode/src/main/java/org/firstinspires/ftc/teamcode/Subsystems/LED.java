package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sarthak on 12/30/2017.
 */

public class LED {

    //Create motor to control LED
    private DcMotor led;
    //Create constants to control LED powers
    private final double POWER_ON = 1;
    private final double POWER_OFF = 0;

    /**
     * Construct LED object
     * @param led DcMotor that controls the LED brightness
     */
    public LED(DcMotor led){
        this.led = led;
    }

    /**
     * Turn on the LEDs
     * @return true, once the action is complete
     */
    public boolean turnOn(){
        setLEDPower(POWER_ON);
        return true;
    }

    /**
     * Turn off the LEDs
     * @return true, once the action is complete
     */
    public boolean turnOff(){
        setLEDPower(POWER_OFF);
        return true;
    }

    /**
     * Set the LEDs to a specific power (brightness)
     * @param power the brightness that the LEDs are set to (range is 0-1)
     * @return true, once the action is complete
     */
    public boolean setLEDPower(double power){
        this.led.setPower(power);
        return true;
    }

}
