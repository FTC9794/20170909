package org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import java.util.Arrays;

/**
 * Created by Sarthak on 10/21/2017.
 */

public class MRRangeSensor implements IUltrasonic {
    private ModernRoboticsI2cRangeSensor ultrasonic;
    private boolean firstRead = false;
    private double prevValue, currentValue;

    public MRRangeSensor(ModernRoboticsI2cRangeSensor ultrasonic){
        this.ultrasonic = ultrasonic;
    }

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
        //return smoothMedian();
    }

    public double opticalDistance() { return ultrasonic.cmOptical(); }

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
