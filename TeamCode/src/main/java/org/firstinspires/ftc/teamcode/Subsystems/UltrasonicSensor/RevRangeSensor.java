package org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Sarthak on 12/25/2017.
 */

public class RevRangeSensor implements IUltrasonic {

    private LynxI2cColorRangeSensor ultrasonic;

    public RevRangeSensor(LynxI2cColorRangeSensor ultrasonic){
        this.ultrasonic = ultrasonic;
    }

    @Override
    public double cmDistance() {
        return ultrasonic.getDistance(DistanceUnit.CM);
    }
}
