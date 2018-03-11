package org.firstinspires.ftc.teamcode.Subsystems.IMU;

import com.kauailabs.navx.ftc.AHRS;

/**
 * Created by Sarthak on 9/24/2017.
 */

public class NavxIMU implements IIMU {

    AHRS navX;
    double offset;

    //Constructor
    public NavxIMU(AHRS NavX){
        this.navX = NavX;
    }

    //Get X Angle
    @Override
    public double getXAngle() {
        return navX.getPitch() ;
    }

    //Get Y angle
    @Override
    public double getYAngle() {
        return navX.getRoll();
    }

    //Get Z Angle
    @Override
    public double getZAngle() {
        return navX.getYaw();
    }

    @Override
    public double getZAngle(double desiredAngle) {
        return 0;
    }

    //Get X Acceleration
    @Override
    public double getXAcc() {
        return navX.getRawAccelX();
    }


    //Get Y Acceleration
    @Override
    public double getYAcc() {
        return navX.getRawAccelY();
    }

    //Get Z Acceleration
    @Override
    public double getZAcc() {
        return navX.getRawAccelZ();
    }

    //Get X Velocity
    @Override
    public double getXVelo() {
        return 0;
    }

    //Get Y Velocity
    @Override
    public double getYVelo() {
        return 0;
    }

    //Get Z Velocity
    @Override
    public double getZVelo() {
        return 0;
    }

    //Calibrate
    @Override
    public void calibrate() {
        while(navX.isCalibrating()){}
    }

    //Set angle offset
    @Override
    public void setOffset(double offset) {
        this.offset = offset;
    }

    //Set the current position as 0
    @Override
    public void setAsZero() {
        navX.zeroYaw();
    }

    //Initialize imu parameters
    @Override
    public void initialize() {

    }
}
