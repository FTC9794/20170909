package org.firstinspires.ftc.teamcode.Subsystems.IMU;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by Sarthak on 9/24/2017.
 */

public class BoschIMU implements IIMU {
    BNO055IMU imu;
    double offset;

    //Constructor
    public BoschIMU(BNO055IMU imu){
        this.imu = imu;
    }

    //Get X Angle
    @Override
    public double getXAngle() {
        return -imu.getAngularOrientation().thirdAngle - offset;
    }

    //Get Y Angle
    @Override
    public double getYAngle() {
        return -imu.getAngularOrientation().secondAngle - offset;
    }

    //Get Z Angle
    @Override
    public double getZAngle() {
        return -imu.getAngularOrientation().firstAngle - offset;
    }

    @Override
    public double getZAngle(double desiredAngle) {
        double angle = getZAngle();
        if(angle<desiredAngle-180){
            angle+=360;
        }else if(angle>desiredAngle+180){
            angle-=360;
        }
        return angle;
    }

    //Get X Acceleration
    @Override
    public double getXAcc() {
        return imu.getAcceleration().xAccel;
    }

    //Get Y Acceleration
    @Override
    public double getYAcc() {
        return imu.getAcceleration().yAccel;
    }

    //Get Z Acceleration
    @Override
    public double getZAcc() {
        return imu.getAcceleration().zAccel;
    }

    //Get X Velocity
    @Override
    public double getXVelo() {
        return imu.getVelocity().xVeloc;
    }

    //Get Y Velocity
    @Override
    public double getYVelo() {
        return imu.getVelocity().yVeloc;
    }

    //Get Z Velocity
    @Override
    public double getZVelo() { return imu.getVelocity().zVeloc; }

    //Calibrate IMU
    @Override
    public void calibrate() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IIMU";
        imu.initialize(parameters);
        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
    }
    //Intialize IMU Parameters
    @Override
    public void initialize(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    //Set an offset
    @Override
    public void setOffset(double offset) {
        this.offset = offset;
    }

    //Set the current position to 0
    @Override
    public void setAsZero() {
        offset = -imu.getAngularOrientation().firstAngle;
    }
}
