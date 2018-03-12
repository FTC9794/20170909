package org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Sarthak on 12/25/2017.
 */

public class RevRangeSensor implements IUltrasonic {

    //Create sensor hardware
    private LynxI2cColorRangeSensor ultrasonic;

    /**
     * Constructor for Rev Color/Range sensor
     * @param ultrasonic Rev Range Sensor hardware
     */
    public RevRangeSensor(LynxI2cColorRangeSensor ultrasonic){
        this.ultrasonic = ultrasonic;
    }

    /**
     * Reads the sensor and returns the distance for the target in centimeters
     * @return the distance in centimeters, as a double
     */
    @Override
    public double cmDistance() {
        return ultrasonic.getDistance(DistanceUnit.CM);
    }
}
