package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DataLogger;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Utilities;

import java.util.Date;
import java.util.List;

/**
 * Created by Sarthak on 10/6/2017.
 */
@Autonomous(name = "Omni Drive Class Test", group = "Test")
public class OmniDirectionalDrive implements IDrivetrain {

    private List<DcMotor> motors;
    private IIMU imu;
    private DataLogger data;
    private AccelerationThread accThread;

    private final double END_ANGLE_OFFSET = 5;

    private ElapsedTime pivotTime;

    private boolean targetReached = false;
    private boolean needsToPivot = false;

    private double rfLastEncoder=0;
    private double rbLastEncoder=0;
    private double lfLastEncoder=0;
    private double lbLastEncoder=0;
    Telemetry telemetry;

    /**
     * Constructor
     *
     * @param motors List of motors on drivetrain in order of Right Front, Right Back, Left Front and then Left Back
     * @param imu the inertial measurement unit or gyro sensor of the robot
     */
    public OmniDirectionalDrive(List<DcMotor> motors, IIMU imu, Telemetry telemetry){
        this.motors = motors;
        this.imu = imu;
        this.imu.calibrate();
        this.telemetry = telemetry;
        data = new DataLogger(new Date().toString()+"Omni Directional");
        pivotTime = new ElapsedTime();
        accThread = new AccelerationThread();
        for(DcMotor motor:motors){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(motor.getClass()==AcceleratedDcMotor.class){
                accThread.addMotor((AcceleratedDcMotor) motor);
            }
        }
        accThread.start();
        data.addField("Slide or Pivot");
        data.addField("high power");
        data.addField("low power");
        data.addField("move angle");
        data.addField("orientation");
        data.addField("power change");
        data.addField("power gain");
        data.addField("o gain");
        data.addField("current angle");
        data.addField("adjusted current angle");
        data.addField("adjusted move angle");
        data.addField("power");
        data.addField("adjusted power");
        data.addField("horizontal");
        data.addField("vertical");
        data.addField("pivot correction");
        data.newLine();

    }

    public OmniDirectionalDrive(List<DcMotor> motors, IIMU imu){
        this.motors = motors;
        this.imu = imu;
        this.imu.calibrate();
        pivotTime = new ElapsedTime();
        accThread = new AccelerationThread();
        for(DcMotor motor:motors){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(motor.getClass()==AcceleratedDcMotor.class){
                accThread.addMotor((AcceleratedDcMotor) motor);
            }
        }
        accThread.start();
    }

    /**
     *
     * @param highPower the maximum power for the robot to move
     * @param lowPower the lowest power for the robot to move
     * @param powerChange however close the robot is to the final target
     * @param powerGain the gain for how much to slow down as the robot approaches the final target
     * @param moveAngle the angle at which the robot will move in the frame of reference of the field
     * @param oGain the gain for the robot to correct the orientation
     * @param pGain the gain at which the robot will correct the orientation when pivoting
     * @param endOrientationAngle the angle at which the robot will end up at and follow while moving
     * @param endCondition whether the robot has reached the desired position
     * @param timeAfterAngle how much the robot should pivot after being +- 5 degrees of the endOrientationAngle
     * @return true if method still needs to run to finish action or false if method is done running
     */
    @Override
    public boolean move(double highPower, double lowPower, double powerChange, double powerGain, double moveAngle, double oGain, double pGain, double endOrientationAngle, boolean endCondition, double timeAfterAngle) {
        if((endCondition&&
                (imu.getZAngle()>endOrientationAngle+END_ANGLE_OFFSET||imu.getZAngle()<endOrientationAngle-END_ANGLE_OFFSET))
                ||needsToPivot){
            needsToPivot = pivotToAngle(endOrientationAngle, highPower, lowPower, timeAfterAngle, pGain);;
            telemetry.addData("endOrientationAngle", endOrientationAngle);
            telemetry.addData("State", "Pivot");
            //telemetry.update();
            return needsToPivot;
        }else if(!endCondition){
            slideAngle(moveAngle, endOrientationAngle, highPower, lowPower, powerChange, powerGain, oGain);
            telemetry.addData("State", "Slide");
            //telemetry.update();
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

        double difference = (currentAngle - angle)+180;
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
            this.setPowerAll(magSpeed, magSpeed, -magSpeed, -magSpeed);
        }else{
            this.setPowerAll(-magSpeed, -magSpeed, magSpeed, magSpeed);
        }
        //send back data about what the robot is doing
        if(currentAngle<angle+END_ANGLE_OFFSET&&currentAngle>angle-END_ANGLE_OFFSET&&!targetReached){
            targetReached = true;
            telemetry.addData("Pivot", "Timer Reset");
            pivotTime.reset();
        }
        if(targetReached&&pivotTime.milliseconds()>timeAfterAngle){
            this.setPowerAll(0, 0, 0, 0);
            targetReached = false;
            return false;
        }else{
            telemetry.addData("Angle Difference", difference);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Angle", angle);
            telemetry.addData("Angle Cond 1", angle+END_ANGLE_OFFSET);
            telemetry.addData("Angle Cond 2", angle-END_ANGLE_OFFSET);
            return true;
        }
    }

    /**
     *
     * @param moveAngle
     * @param orientationAngle
     * @param highPower
     * @param lowPower
     * @param powerPercent
     * @param powerGain
     * @param oGain
     */
    public void slideAngle(double moveAngle, double orientationAngle, double highPower, double lowPower, double powerPercent, double powerGain, double oGain) {
        data.addField("Slide");
        data.addField((float)highPower);
        data.addField((float)lowPower);
        data.addField((float)moveAngle);
        data.addField((float)orientationAngle);
        data.addField((float)powerPercent);
        data.addField((float)powerGain);
        data.addField((float)oGain);
        double currentAngle = imu.getZAngle();
        data.addField((float)currentAngle);
        boolean fixAngle = true;

        while(fixAngle){
            if(currentAngle>orientationAngle+180){
                currentAngle = currentAngle-360;
            }else if(currentAngle<orientationAngle-180){
                currentAngle = currentAngle+360;
            }else{
                fixAngle = false;
            }
        }
        data.addField((float)currentAngle);
        telemetry.addData("imu", currentAngle);
        moveAngle = moveAngle - orientationAngle;
        if(moveAngle <= -180){
            moveAngle+=360;
        }
        data.addField((float)moveAngle);
        telemetry.addData("move angle", moveAngle);
        double power = powerPercent*powerGain;
        data.addField((float)power);
        if(power<0){
            if(power<-highPower){
                power = -highPower;
            }
            if(power>-lowPower){
                power = -lowPower;
            }
        }else{
            if(power>highPower){
                power = highPower;
            }
            if(power<lowPower){
                power = lowPower;
            }
        }
        data.addField((float)power);
        double horizontal = Utilities.round2D(calculateX(moveAngle, power));
        data.addField((float)horizontal);
        double vertical = Utilities.round2D(calculateY(moveAngle, power));
        data.addField((float)vertical);
        double pivotCorrection = -((orientationAngle - currentAngle) * oGain);
        data.addField((float)pivotCorrection);
        //data.newLine();
        rawSlide(horizontal, vertical, pivotCorrection, power);
        data.newLine();
    }

    /**
     * Sets power to each of the motors
     * @param rfPower the power set to the Right Front motor
     * @param rbPower the power set to the Right Back motor
     * @param lfPower the power set to the Left Front motor
     * @param lbPower the power set to the Left Back motor
     */
    private void setPowerAll(double rfPower, double rbPower, double lfPower, double lbPower){
        motors.get(0).setPower(rfPower);
        motors.get(1).setPower(rbPower);
        motors.get(2).setPower(lfPower);
        motors.get(3).setPower(lbPower);
    }

    public void rawSlide(double horizontal, double vertical, double pivot, double speed){
        //create an array with all the speeds
        double speeds[] = {vertical-horizontal+pivot, vertical+horizontal+pivot, vertical+horizontal-pivot,vertical-horizontal-pivot};

        //Only adjust speeds if the robot is moving
        if(horizontal!=0 || vertical!=0){
            int max = 0;
            int counter = 0;

            //determine the maximum speed out of the four motors
            for(double element:speeds){
                if(Math.abs(element)>Math.abs(speeds[max])){
                    max = counter;
                }
                counter++;
            }

            //set the maximum as a variable
            double maxSpeed = Math.abs(speeds[max]);

            //divide all of the speeds by the max speed to make sure that
            if(maxSpeed!=0){
                speeds[0]=speeds[0]/maxSpeed*speed;
                speeds[1]= (speeds[1]/maxSpeed*speed);
                speeds[2]= (speeds[2]/maxSpeed*speed);
                speeds[3]=speeds[3]/maxSpeed*speed;
                data.addField((float) speeds[0]);
                data.addField((float) speeds[1]);
                data.addField((float) speeds[2]);
                data.addField((float) speeds[3]);
            }
        }

        //set all the powers and display telemetry data
        this.setPowerAll(speeds[0], speeds[1], speeds[2], speeds[3]);
    }
    //returns X vector value using angle and speed
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    //returns the Y vector value using angle and speed
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    //Resets encoder values to zero
    @Override
    public void resetEncoders(){
        for(DcMotor motor:motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //Sets base encoder value to the current position of the motors
    @Override
    public void softResetEncoder(){
        rfLastEncoder = motors.get(0).getCurrentPosition();
        lfLastEncoder = motors.get(1).getCurrentPosition();
        rbLastEncoder = motors.get(2).getCurrentPosition();
        lbLastEncoder = motors.get(3).getCurrentPosition();
    }


    //Returns an array with encoder positions for each drive motor
    private double[] getEncoderPositions(){
        double[] encoders = {motors.get(0).getCurrentPosition()-rfLastEncoder, motors.get(1).getCurrentPosition()-rbLastEncoder, motors.get(2).getCurrentPosition()-lfLastEncoder, motors.get(3).getCurrentPosition()-lbLastEncoder};
        return encoders;
    }

    //Returns the current distance traveled/encoder position by averaging the position of each drive motor
    @Override
    public double averageEncoders(){
        double []encoders = this.getEncoderPositions();
        double encoderA = (encoders[2] + encoders[1]) / 2;
        double encoderB = (encoders[0] + encoders[3]) / 2;
        double encoderAverage = (Math.abs(encoderA) + Math.abs(encoderB)) / 2;
        return encoderAverage;
    }
    /**
     * Turns off the drivetrain and stops all threads related to the driveetrain
     */
    @Override
    public void stop() {
        accThread.accelerating=false;
        this.setPowerZero();
        accThread.stop();
    }

    //Set zero power to all drive motors
    @Override
    public void setPowerZero(){
        setPowerAll(0, 0, 0, 0);
    }
}
