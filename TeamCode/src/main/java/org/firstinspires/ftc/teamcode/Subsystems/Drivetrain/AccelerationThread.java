package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Created by ishaa on 7/2/2017.
 */

public class AccelerationThread implements Runnable {
    ArrayList<AcceleratedDcMotor> motors = new ArrayList<AcceleratedDcMotor>();
    boolean accelerating = true;
    public boolean controled = false;
    ElapsedTime timer;
    Thread accelerationControlThread;

    public AccelerationThread(){

    }

    public void addMotor(AcceleratedDcMotor motor){
        motors.add(motor);
    }

    public synchronized void start(){
        timer = new ElapsedTime();
        if(!controled){
            controled = true;
            accelerationControlThread = new Thread(this);
            accelerationControlThread.start();
        }
    }
    @Override
    public synchronized void run() {
        while(controled){
            timer.reset();
            if(accelerating){
                for(AcceleratedDcMotor motor:motors){
                    motor.update();
                }
            }else{
                for(AcceleratedDcMotor motor:motors){
                    motor.updateRawPower();
                }
            }
            while(timer.milliseconds()<100){

            }

        }

    }

    public synchronized void isAccelerating(boolean accelerating){
        this.accelerating = accelerating;
    }
    public void stop(){
        controled = false;
        for(AcceleratedDcMotor motor:motors){
            motor.stopMotor();
        }
    }
}
