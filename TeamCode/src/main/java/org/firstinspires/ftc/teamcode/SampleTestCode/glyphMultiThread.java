package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntakeThread;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.RevRangeSensor;

/**
 * Created by ishaa on 3/3/2018.
 */

@TeleOp(name="glyph thread")
public class glyphMultiThread extends LinearOpMode {
    CRServo rightWheel1, rightWheel2, leftWheel1, leftWheel2;
    LynxI2cColorRangeSensor glyphSensor1, glyphSensor2;
    LynxI2cColorRangeSensor glyphColor1, glyphColor2;
    DualWheelIntakeThread bottomIntake, topIntake;
    Thread topIntakethread, bottomIntakeThread;
    boolean xPressed;

    @Override
    public void runOpMode() throws InterruptedException {
        rightWheel1 = hardwareMap.crservo.get("right_glyph1");
        leftWheel1 = hardwareMap.crservo.get("left_glyph1");
        rightWheel2 = hardwareMap.crservo.get("right_glyph2");
        leftWheel2 = hardwareMap.crservo.get("left_glyph2");


        //Set servo behaviors
        leftWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        rightWheel1.setDirection(DcMotorSimple.Direction.REVERSE);

        //Hardware Map Color Sensors
        glyphColor1 = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor1");
        glyphColor2 = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor2");

        //glyphColor1 = new RevRangeSensor(glyphSensor1);
        //glyphColor2 = new RevRangeSensor(glyphSensor2);

        bottomIntake = new DualWheelIntakeThread(rightWheel1, leftWheel1, glyphColor1, telemetry);
        topIntake = new DualWheelIntakeThread(rightWheel2, leftWheel2, glyphColor2, telemetry);

        topIntakethread = new Thread(topIntake);
        bottomIntakeThread = new Thread(bottomIntake);


        xPressed = false;
        waitForStart();

        telemetry.addData("run", "running");
        telemetry.update();

        topIntakethread.start();
        bottomIntakeThread.start();

        topIntake.secureGlyph();
        //topIntake.secureGlyph();
/*        while(!isStopRequested()){
            if(gamepad1.x){
                bottomIntake.secureGlyph();
                topIntake.secureGlyph();
                telemetry.addData("xPressed", true);
                telemetry.update();
            }else if(gamepad1.y){
                bottomIntake.dispenseGlyph();
                topIntake.dispenseGlyph();
            }else if(gamepad1.a){
                bottomIntake.turnOff();
                topIntake.turnOff();
            }


        }*/
        bottomIntake.stop();
        topIntake.stop();

    }
}