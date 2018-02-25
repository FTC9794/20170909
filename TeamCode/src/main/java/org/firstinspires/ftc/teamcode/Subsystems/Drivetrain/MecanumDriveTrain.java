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
public class MecanumDriveTrain implements IDrivetrain {

    //hardware of drivetrain
    private List<DcMotor> motors;
    private IIMU imu;

    private DataLogger data;

    //How much the robot can be off the desired angle at the end of a motion
    private final double END_ANGLE_OFFSET = 5;

    private ElapsedTime pivotTime;

    private boolean targetReached = false;
    private boolean needsToPivot = false;

    //Variables where last encoder value when reset are stored
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
    public MecanumDriveTrain(List<DcMotor> motors, IIMU imu, Telemetry telemetry){
        this.motors = motors;
        this.imu = imu;
        this.imu.initialize();
        this.telemetry = telemetry;
        pivotTime = new ElapsedTime();
    }

    //No IMU Here
    public MecanumDriveTrain(List<DcMotor> motors, Telemetry telemetry){
        this.motors = motors;
        this.imu = null;
        this.telemetry = telemetry;
        pivotTime = new ElapsedTime();
    }
    /**
     *
     * @param angle the angle at which the robot will pivot to in the frame of reference of the driver
     * @param maxPower the maximum speed that the robot will travel
     * @param lowPower the minimum speed of the robot
     * @param timeAfterAngle the time the robot will spend to correct once +- 5 degrees away from target angle
     * @param pGain the amount of speed applied per degree the robot is away from the target angle
     * @return true if the pivot is still occurring false when it is complete
     */
    private boolean pivotToAngle(double angle, double maxPower, double lowPower, double timeAfterAngle, double[] pGain) {
        //get gyro sensor value
        double currentAngle = imu.getZAngle(angle);

        //determine how far the current angle is from the target angle
        double difference = (currentAngle - angle);

        //using PID feedback to determine power determine how much power to apply, just p right now
        double power = Math.abs(difference * pGain[0]);

        //shrink power to fit min and max
        if(power > maxPower){
            power = maxPower;
        }
        if(power<lowPower){
            power = lowPower;
        }

        //pivot left or right depending on which side of target angle
        if(difference<0){
            this.setPowerAll(-power, -power, power, power);
        }else{
            this.setPowerAll(power, power, -power, -power);
        }

        //Once the robot angle is within plus or minus angle offset, start a timer and set target reached to true
        if(currentAngle<angle+END_ANGLE_OFFSET&&currentAngle>angle-END_ANGLE_OFFSET&&!targetReached){
            targetReached = true;
            pivotTime.reset();
        }

        //check if target reached is true and if the timer has exceeded the amount of milliseconds desired
        if(targetReached&&pivotTime.milliseconds()>timeAfterAngle){
            this.stop();
            targetReached = false;
            return false;
        }else{
            return true;
        }
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

    //Takes the horizontal power, vertical power and pivoting power and determines how much power to apply to each wheel and normalizes to max power
    public void rawSlide(double horizontal, double vertical, double pivot, double maxPower){
        //create an array with all the speeds
        double powers[] = {vertical-horizontal+pivot, vertical+horizontal+pivot, vertical+horizontal-pivot,vertical-horizontal-pivot};

        //Only adjust speeds if the robot is moving
        if(horizontal!=0 || vertical!=0){
            int max = 0;
            int counter = 0;

            //determine the maximum speed out of the four motors
            for(double element:powers){
                if(Math.abs(element)>Math.abs(powers[max])){
                    max = counter;
                }
                counter++;
            }

            //set the maximum as a variable
            double maxCalculatedPower = Math.abs(powers[max]);

            //divide all of the speeds by the max speed to make sure that
            if(maxCalculatedPower!=0){
                powers[0]=powers[0]/maxCalculatedPower*maxPower;
                powers[1]=powers[1]/maxCalculatedPower*maxPower;
                powers[2]=powers[2]/maxCalculatedPower*maxPower;
                powers[3]=powers[3]/maxCalculatedPower*maxPower;

            }
        }
        //set all the powers and display telemetry data
        this.setPowerAll(powers[0], powers[1], powers[2], powers[3]);
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
        rfLastEncoder = 0;
        rbLastEncoder = 0;
        lfLastEncoder = 0;
        lbLastEncoder = 0;
    }

    /**
     *
     * @param currentPosition The current position of robot in any unit eg. encoder counts, sensor distances...
     * @param targetPosition The target position of the robot in the same unit as current position
     * @param rampDownTargetPosition Position at which the robot will start ramping down
     * @param rampUpTargetPosition Position at which the robot will stop ramping up in power
     * @param maxPower The maximum power the robot will move at
     * @param lowPower The lowest power the robot will move at
     * @param moveAngle The angle at which the robot will move in the frame of reference of the starting position
     * @param PIDGain Three gains to control PID feedback loop for Orientation correction
     * @param endOrientationAngle The direction the robot is facing
     * @return
     */
    @Override
    public boolean moveIMU(double currentPosition, double targetPosition, double rampDownTargetPosition, double rampUpTargetPosition, double maxPower, double lowPower, double moveAngle, double[] PIDGain, double endOrientationAngle, double allowableDistanceError) {
        if(currentPosition < targetPosition){
            double currentAngle = imu.getZAngle(endOrientationAngle);
            moveAngle = moveAngle - endOrientationAngle;
            if(moveAngle <= -180){
                moveAngle+=360;
            }
            double power;
            //ramp up ramp, down or stay flat power
            if(currentPosition<rampUpTargetPosition){
                power = (maxPower-lowPower)/(rampUpTargetPosition)*currentPosition+lowPower;
            }else if(currentPosition>rampDownTargetPosition){
                power = (lowPower-maxPower)/(targetPosition-rampDownTargetPosition)*(currentPosition-rampDownTargetPosition)+maxPower;
            }else{
                power = maxPower;
            }

            double horizontal = Utilities.round2D(calculateX(moveAngle, power));
            double vertical = Utilities.round2D(calculateY(moveAngle, power));
            double pivotCorrection = ((currentAngle - endOrientationAngle) * PIDGain[0]);
            rawSlide(horizontal, vertical, pivotCorrection, power);
            return true;
        } else if(currentPosition > targetPosition + allowableDistanceError){
            double currentAngle = imu.getZAngle(endOrientationAngle);
            moveAngle = moveAngle - endOrientationAngle + 180;
            if(moveAngle <= -180){
                moveAngle+=360;
            }
            double power;
            //ramp up ramp, down or stay flat power
            power  = lowPower;
            double horizontal = Utilities.round2D(calculateX(moveAngle, power));
            double vertical = Utilities.round2D(calculateY(moveAngle, power));
            double pivotCorrection = ((currentAngle - endOrientationAngle) * PIDGain[0]);
            rawSlide(horizontal, vertical, pivotCorrection, power);
            return true;
        }
        else{
            setPowerAll(0, 0, 0, 0);
            return false;
        }
    }

    @Override
    public boolean pivotIMU(double desiredAngle, double rampDownAngle, double maxPower, double minPower, double correctionTime) {
        double currentAngle = imu.getZAngle(desiredAngle);
        double difference = desiredAngle-currentAngle;
        double rampDownDifference = desiredAngle - rampDownAngle;
        double power;
        if(Math.abs(difference)>Math.abs(rampDownAngle)){
            power = maxPower;
        }
        return false;
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
        double x = ((encoders[2] + encoders[1]) - (encoders[0] + encoders[3])) / 4;
        double y = (encoders[0] + encoders[1] + encoders[2] + encoders[3]) / 4;
        double encoderAverage = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        return encoderAverage;
    }

    @Override
    public double getEncoderDistance() {
        return 0;
    }

    /**
     * Turns off the drivetrain and stops all threads related to the driveetrain
     */
    @Override
    public void stop() {
        setPowerAll(0, 0, 0, 0);
    }
}
