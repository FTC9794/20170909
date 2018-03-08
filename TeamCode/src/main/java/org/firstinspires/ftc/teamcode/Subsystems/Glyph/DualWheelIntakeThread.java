package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Enums.GlyphIntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.RevRangeSensor;

/**
 * Created by ishaa on 3/2/2018.
 */

public class DualWheelIntakeThread implements Runnable {
    GlyphIntakeState state;
    CRServo servo1, servo2;
    boolean running;
    ElapsedTime glyphTimer;
    RevRangeSensor intakeSensor;
    boolean glyphSeen = false;
    Telemetry telemetry;
    double x =0;

    public DualWheelIntakeThread(CRServo servo1, CRServo servo2, RevRangeSensor intakeSensor, Telemetry telemetry){
        this.servo1 = servo1;
        this.servo2 = servo2;
        glyphTimer = new ElapsedTime();
        this.intakeSensor = intakeSensor;
        state = GlyphIntakeState.NOTHING;
        this.telemetry = telemetry;
        running = true;
    }

    @Override
    public void run() {

        while(running){


            switch(state){
                case NOTHING:
                    servo1.setPower(0);
                    servo2.setPower(0);
                    telemetry.addData("State", "nothing");

                    break;

                case INTAKE_MOTOR:
                    if(intakeSensor.cmDistance()<6&&!glyphSeen){
                        glyphSeen = true;
                        glyphTimer.reset();
                    }
                    if(glyphSeen&&glyphTimer.milliseconds()>250){
                        state = GlyphIntakeState.INTAKE_NO_MOTOR;
                        glyphSeen = false;
                    }
                    else{
                        servo1.setPower(.7);
                        servo2.setPower(.7);
                    }
                    telemetry.addData("State", "Intake");
                    break;
                case INTAKE_NO_MOTOR:
                    if(intakeSensor.cmDistance()>6){
                        state = GlyphIntakeState.INTAKE_MOTOR;
                    }else{
                        servo1.setPower(0);
                        servo2.setPower(0);
                    }
                    break;
                case OUTAKE:
                    servo1.setPower(-.7);
                    servo2.setPower(-.7);
                    telemetry.addData("State", "outake");
                    break;
            }
            telemetry.update();
        }
    }


    public void secureGlyph() {
        state = GlyphIntakeState.INTAKE_MOTOR;
    }

    public void dispenseGlyph() {
        state = GlyphIntakeState.OUTAKE;
    }

    public void turnOff() {
        state = GlyphIntakeState.NOTHING;
    }

    public void stop(){
        servo1.setPower(0);
        servo2.setPower(0);
        running = false;
        telemetry.addData("stopp", "STOPPED");
    }
}
