package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public DualWheelIntakeThread(CRServo servo1, CRServo servo2, RevRangeSensor intakeSensor){
        this.servo1 = servo1;
        this.servo2 = servo2;
        glyphTimer = new ElapsedTime();
        this.intakeSensor = intakeSensor;
        state = GlyphIntakeState.NOTHING;

    }

    @Override
    public void run() {
        running = true;
        while(running){
            switch(state){
                case NOTHING:
                    servo1.setPower(0);
                    servo2.setPower(0);
                    break;

                case INTAKE_MOTOR:
                    if(intakeSensor.cmDistance()<6&&!glyphSeen){
                        glyphSeen = true;
                        glyphTimer.reset();
                    }
                    if(glyphSeen&&glyphTimer.milliseconds()>250){
                        state = GlyphIntakeState.INTAKE_NO_MOTOR;
                        glyphSeen = false;
                    }else{
                        servo1.setPower(1);
                        servo2.setPower(1);
                    }
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
                    servo1.setPower(-1);
                    servo2.setPower(-1);
                    break;
            }
        }
    }


    public synchronized void secureGlyph() {
        state = GlyphIntakeState.INTAKE_MOTOR;
    }


    public synchronized void dispenseGlyph() {
        state = GlyphIntakeState.OUTAKE;
    }

    public synchronized void turnOff() {
        state = GlyphIntakeState.NOTHING;
    }

    public synchronized void stop(){
        servo1.setPower(0);
        servo2.setPower(0);
        running = false;
    }
}
