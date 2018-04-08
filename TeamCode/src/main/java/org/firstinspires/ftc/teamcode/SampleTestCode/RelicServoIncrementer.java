package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sarthak on 4/1/2018.
 */
@TeleOp(name = "Relic Servo Incrementer", group = "Relic")
public class RelicServoIncrementer extends LinearOpMode {
    //Servo Variables
    Servo relic_claw, relic_arm, relic_tilt;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        //Set Servo Init Positions
        relic_tilt.setPosition(1);
        relic_arm.setPosition(0);

        telemetry.addData("Relic Servo Arm Controls", "A (+) & Y (-)");
        telemetry.addData("Relic Servo Tilt Controls", "B (+) & X (-)");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();

        while(opModeIsActive()){

            //Arm increment controls
            if(gamepad1.a){
                if(relic_arm.getPosition() < 0.999){
                    relic_arm.setPosition(relic_arm.getPosition() + 0.001);
                }
            }else if(gamepad1.y){
                if(relic_arm.getPosition() > 0.001){
                    relic_arm.setPosition(relic_arm.getPosition() - 0.001);
                }
            }

            //Tilt Increment Controls
            if(gamepad1.x){
                if(relic_tilt.getPosition() < 0.999){
                    relic_tilt.setPosition(relic_tilt.getPosition() + 0.001);
                }
            }else if(gamepad1.b){
                if(relic_tilt.getPosition() > 0.001){
                    relic_tilt.setPosition(relic_tilt.getPosition() - 0.001);
                }
            }

            telemetry.addData("Relic Servo Arm Position", relic_arm.getPosition());
            telemetry.addData("Relic Servo Tilt Position", relic_tilt.getPosition());
            telemetry.update();
        }

    }


    /**
     * Hardware maps all motors, servos, and sensors
     */
    public void initHardwareMap(){
        //Hardware map servos
        relic_arm = hardwareMap.servo.get("relic_arm");
        relic_claw = hardwareMap.servo.get("relic_claw");
        relic_tilt = hardwareMap.servo.get("relic_tilt");


    }

}
