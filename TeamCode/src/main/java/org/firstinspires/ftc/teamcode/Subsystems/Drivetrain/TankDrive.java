package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;

import java.util.List;

/**
 * Created by Sarthak on 9/24/2017.
 */

public class TankDrive implements IDrivetrain {
    List<DcMotor> motors;
    IIMU imu;
    AccelerationThread accThread;
    private final double END_ANGLE_OFFSET = 5;
    private ElapsedTime pivotTime;

    private boolean needsToPivot = false;
    private boolean targetReached = false;

    private double rfLastEncoder=0;
    private double rbLastEncoder=0;
    private double lfLastEncoder=0;
    private double lbLastEncoder=0;

    public TankDrive(List<DcMotor> motors, IIMU imu){
        pivotTime = new ElapsedTime();
        this.motors = motors;
        this.imu = imu;
        accThread = new AccelerationThread();
        for(DcMotor motor:motors){
            if(motor.getClass()==AcceleratedDcMotor.class){
                accThread.addMotor((AcceleratedDcMotor) motor);
            }
        }
        accThread.start();
    }


    @Override
    public boolean move(double highPower, double lowPower, double powerChange, double powerGain, double moveAngle, double oGain, double pGain, double endOrientationAngle, boolean endCondition, double timeAfterAngle) {
        if((endCondition&&
                (imu.getZAngle()>endOrientationAngle+END_ANGLE_OFFSET||imu.getZAngle()<endOrientationAngle-END_ANGLE_OFFSET))
                ||needsToPivot){
            needsToPivot = pivotToAngle(endOrientationAngle, highPower, lowPower, timeAfterAngle, pGain);;
            return needsToPivot;
        }else if(!endCondition){
            slideAngle(highPower, lowPower, powerChange, powerGain, moveAngle, oGain);
            return true;
        }else{
            needsToPivot = false;
            return false;
        }
    }

    @Override
    public boolean balance(double highPower, double lowPower, double moveAngle, double oGain, boolean endCondition) {
        return false;
    }

    public void slideAngle(double highPower, double lowPower, double powerChange, double powerGain, double moveAngle, double oGain){
        double currentAngle = imu.getZAngle();
        boolean fixAngle = true;
        while(fixAngle){
            if(currentAngle>moveAngle+180){
                currentAngle = currentAngle-360;
            }else if(currentAngle<moveAngle-180){
                currentAngle = currentAngle+360;
            }else{
                fixAngle = false;
            }
        }
        double rightPower = powerChange*powerGain-oGain*(moveAngle-currentAngle);
        double leftPower = powerChange*powerGain+oGain*(moveAngle-currentAngle);

        if(rightPower>0){
            rightPower = Math.max(Math.min(rightPower, highPower), lowPower);
        }else{
            rightPower = Math.min(Math.max(rightPower, -highPower), -lowPower);
        }
        if(leftPower>0){
            leftPower = Math.max(Math.min(leftPower, highPower), lowPower);
        }else{
            leftPower = Math.min(Math.max(leftPower, -highPower), -lowPower);
        }

        this.setPowerAll(rightPower, leftPower);
    }
    /**
     *
     * @param angle the angle at which the robot will pivot to in the frame of reference of the driver
     * @param highPower the maximum speed that the robot will travel
     * @param lowPower the minimum speed of the robot
     * @param timeAfterAngle the time the robot will spend to correct once +- 5 degrees away from target angle
     * @param pGain the amount of speed applied per degree the robot is away from the target angle
     * @return true if the pivot is still occurring false when it is complete
     */
    private boolean pivotToAngle(double angle, double highPower, double lowPower, double timeAfterAngle, double pGain) {
        //measure the gyro sensor
        //get gyro sensor value
        double currentAngle = imu.getZAngle();
        double difference = angle - currentAngle;
        if(difference>180){
            difference-=360;
        }else if(difference<-180){
            difference+=360;
        }

        double magSpeed = Math.abs(difference * pGain);
        if(magSpeed > highPower){
            magSpeed = highPower;
        }
        if(magSpeed<lowPower){
            magSpeed = lowPower;
        }
        if(difference<0){
            this.setPowerAll(magSpeed, -magSpeed);
        }else{
            this.setPowerAll(-magSpeed, magSpeed);
        }
        //send back data about what the robot is doing
        if(currentAngle<angle+END_ANGLE_OFFSET&&currentAngle>angle-END_ANGLE_OFFSET&&!targetReached){
            targetReached = true;
            pivotTime.reset();
        }
        if(targetReached&&pivotTime.milliseconds()>timeAfterAngle){
            this.setPowerAll(0, 0);
            targetReached = false;
            return false;
        }else{
            return true;
        }
    }
    private void setPowerAll(double rightSpeed, double leftSpeed){
        motors.get(0).setPower(rightSpeed);
        motors.get(1).setPower(rightSpeed);
        motors.get(2).setPower(leftSpeed);
        motors.get(3).setPower(leftSpeed);
    }

    //Sets base encoder value to the current position of the motors
    @Override
    public void softResetEncoder() {
        rfLastEncoder = motors.get(0).getCurrentPosition();
        lfLastEncoder = motors.get(1).getCurrentPosition();
        rbLastEncoder = motors.get(2).getCurrentPosition();
        lbLastEncoder = motors.get(3).getCurrentPosition();
    }

    //Resets encoder values to zero
    @Override
    public void resetEncoders() {
        for(DcMotor motor:motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        rfLastEncoder = 0;
        rbLastEncoder = 0;
        lfLastEncoder = 0;
        lbLastEncoder = 0;
    }

    //Returns an array with encoder positions for each drive motor
    private double[] getEncoderPositions(){
        double[] encoders = {motors.get(0).getCurrentPosition()-rfLastEncoder, motors.get(1).getCurrentPosition()-rbLastEncoder, motors.get(2).getCurrentPosition()-lfLastEncoder, motors.get(3).getCurrentPosition()-lbLastEncoder};
        return encoders;
    }

    //Returns the current distance traveled/encoder position by averaging the position of each drive motor
    @Override
    public double averageEncoders() {
        double[] encoder = this.getEncoderPositions();
        double sum = 0;
        for(double position:encoder){
            sum += position;
        }
        return sum/4;
    }

    //Stops drivetrain and threads related to the drivetrain
    @Override
    public void stop() {
        accThread.stop();
    }

    //Set zero power to all drive motors
    @Override
    public void setPowerZero(){
        setPowerAll(0, 0);
    }
}
