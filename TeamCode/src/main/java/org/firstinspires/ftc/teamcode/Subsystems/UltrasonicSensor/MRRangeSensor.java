package org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import java.util.Arrays;

/**
 * Created by Sarthak on 10/21/2017.
 */

public class MRRangeSensor implements IUltrasonic {
    //Create ultrasonic
    private ModernRoboticsI2cRangeSensor ultrasonic;
    //Create instance variables
    private boolean firstRead = false;
    private double prevValue, currentValue;

    //Constructor
    public MRRangeSensor(ModernRoboticsI2cRangeSensor ultrasonic){
        this.ultrasonic = ultrasonic;
    }

    /**
     * Returns the distance read by the sensor
     * @return the distance in centimeters, as a double type
     */
    @Override
    public double cmDistance() {
        if(!firstRead){
            prevValue = ultrasonic.cmUltrasonic();
            firstRead = true;
        }else{
            currentValue = lowPass(ultrasonic.cmUltrasonic(), 0.75, prevValue);
            prevValue = currentValue;
        }
        return currentValue;
    }

    //Returns the optical distance from the sensor (for close-up targets)
    public double opticalDistance() { return ultrasonic.cmOptical(); }

    /**
     * Sensor filter that calculates the median value of five sensor readings
     * @return the filtered ultrasonic value as an integer
     */
    private int smoothMedian(){
        double[] values = new double[5];
        for(int i = 0; i < values.length; i++){
            values[i] = ultrasonic.cmUltrasonic();
        }
        Arrays.sort(values);
        double median;
        if (values.length % 2 == 0) {
            median = ((double) values[values.length / 2] + (double) values[values.length / 2 - 1]) / 2;
        }
        else {
            median = (double) values[values.length / 2];
        }
        return (int) median;
    }

    /**
     * Low pass sensor filter to reduce the effect of spikes in the data
     * @param sensorReading current value the sensor returned
     * @param filterValue gain to multiply the reading by
     * @param smoothedValue previous filtered value from sensor
     * @return
     */
    private int lowPass(double sensorReading, double filterValue, double smoothedValue){
        if(filterValue >= 1){
            filterValue = 0.99;
        }else if(filterValue <= 0){
            filterValue = 0;
        }

        smoothedValue = (sensorReading * (1-filterValue)) + (smoothedValue*filterValue);
        return (int) smoothedValue;
    }
}
