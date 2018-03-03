package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntakeThread;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.RevRangeSensor;

/**
 * Created by ishaa on 3/3/2018.
 */

public class glyphMultiThread extends LinearOpMode {
    CRServo rightWheel1, rightWheel2, leftWheel1, leftWheel2;
    LynxI2cColorRangeSensor glyphSensor1, glyphSensor2;
    RevRangeSensor glyphColor1, glyphColor2;
    DualWheelIntakeThread bottomIntake;
    DualWheelIntakeThread topIntake;

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
        glyphSensor1 = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor1");
        glyphSensor2 = (LynxI2cColorRangeSensor) hardwareMap.get("glyphColor2");

        glyphColor1 = new RevRangeSensor(glyphSensor1);
        glyphColor2 = new RevRangeSensor(glyphSensor2);

        bottomIntake = new DualWheelIntakeThread(rightWheel1, leftWheel1, glyphColor1);
        topIntake = new DualWheelIntakeThread(rightWheel2, leftWheel2, glyphColor2);

        waitForStart();
        bottomIntake.start();
        topIntake.start();

        while(opModeIsActive()){
            if(gamepad1.a){
                bottomIntake.secureGlyph();
                topIntake.secureGlyph();
            }else if(gamepad1.b){
                bottomIntake.dispenseGlyph();
                topIntake.dispenseGlyph();
            }else if(gamepad1.x){
                bottomIntake.turnOff();
                topIntake.turnOff();
            }
        }
        bottomIntake.stop();
        topIntake.stop();
    }
}
