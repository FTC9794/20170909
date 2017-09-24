package org.firstinspires.ftc.teamcode;

/**
 * Created by Sarthak on 9/24/2017.
 */

public interface IIMU {
    double getXAngle();  //Gets angle on x-axis
    double getYAngle();  //Gets angle on y-axis
    double getZAngle();  //Gets angle on z-axis
    double getXAcc();    //Gets acceleration on x-axis
    double getYAcc();    //Gets acceleration on y-axis
    double getZAcc();    //Gets acceleration on z-axis
    double getXVel();    //Gets velocity on x-axis
    double getYVel();    //Gets velocity on y-axis
    double getZVel();    //Gets velocity on z-axis
    void calibrate();    //Calibrates sensor
    void setOffset(double offset);   //sets offset
    void setAsZero();    //Set the current position as zero
}
