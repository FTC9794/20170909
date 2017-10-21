package org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

/**
 * Created by Sarthak on 10/21/2017.
 */

public class MRRangeSensor implements IUltrasonic {
    ModernRoboticsI2cRangeSensor ultrasonic;

    public MRRangeSensor(ModernRoboticsI2cRangeSensor ultrasonic){
        this.ultrasonic = ultrasonic;
    }

    @Override
    public double cmDistance() {
        return ultrasonic.cmUltrasonic();
    }
}
