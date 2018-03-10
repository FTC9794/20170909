package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Enums.GlyphIntakeState;
import org.firstinspires.ftc.teamcode.Subsystems.UltrasonicSensor.RevRangeSensor;

/**
 * Created by ishaa on 3/2/2018.
 */

public class DualWheelIntakeThread implements Runnable, IGlyph {

    final double GLYPH_SEEN_DISTANCE = 6;

    //Create objects to be used in thread
    GlyphIntakeState state;
    CRServo servo1, servo2;
    boolean running;
    ElapsedTime glyphTimer;
    LynxI2cColorRangeSensor intakeSensor;
    boolean glyphSeen = false;
    Telemetry telemetry;

    /**
     * Constructor for Glyph Intake Thread. Class is only for either bottom or top intake
     * @param servo1 The first servo that is a part of this intake
     * @param servo2 The second servo that is a part of the intake
     * @param intakeSensor The sensor that will detect whether a glyph is inside the intake
     * @param telemetry Used for debugging
     */
    public DualWheelIntakeThread(CRServo servo1, CRServo servo2, LynxI2cColorRangeSensor intakeSensor, Telemetry telemetry){
        //set parameters to local variables
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.intakeSensor = intakeSensor;
        this.telemetry = telemetry;

        //initialize objects and variables
        state = GlyphIntakeState.NOTHING;
        glyphTimer = new ElapsedTime();
        running = true;
    }

    @Override
    public void run() {

        //What happens when the thread is started and is running
        while(running){

            //determine what state the intake is in
            switch(state){

                //This state is if the intake isn't doing anything or is off
                case NOTHING:
                    servo1.setPower(0);
                    servo2.setPower(0);
                    break;

                //This state is to intake glyphs
                case INTAKE_MOTOR:

                    //determine if glyph is inside mechanism
                    if(intakeSensor.getDistance(DistanceUnit.CM)<GLYPH_SEEN_DISTANCE&&!glyphSeen){
                        glyphSeen = true;
                        glyphTimer.reset();
                    }else if(intakeSensor.getDistance(DistanceUnit.CM)>GLYPH_SEEN_DISTANCE){
                        glyphSeen = false;
                    }

                    //determine if glyph is inside mechanism and it has been 250 ms since the glyph was seen
                    if(glyphSeen&&glyphTimer.milliseconds()>250){

                        //Leave this state to turn off motors
                        state = GlyphIntakeState.INTAKE_NO_MOTOR;
                        glyphSeen = false;
                    }
                    else{

                        //otherwise intake
                        servo1.setPower(.7);
                        servo2.setPower(.7);
                    }

                    break;

                //This state is if the robot has seen a glyph so the intake motors need to be off, but if the glyph leaves, it intakes
                case INTAKE_NO_MOTOR:

                    //determine whether a glyph is still in the intake
                    if(intakeSensor.getDistance(DistanceUnit.CM)>GLYPH_SEEN_DISTANCE){
                        //go to the state which will turn the motors on
                        state = GlyphIntakeState.INTAKE_MOTOR;
                    }else{
                        //turn off the intake
                        servo1.setPower(0);
                        servo2.setPower(0);
                    }
                    break;

                //this state is to eject glyphs
                case OUTAKE:

                    //eject glyphs
                    servo1.setPower(-.7);
                    servo2.setPower(-.7);
                    break;
            }

        }
    }

    //This method sets the state of the intake to intaking
    public void secureGlyph() {
        state = GlyphIntakeState.INTAKE_MOTOR;
    }

    //this method ejects glyphs by changing the state to outaking
    public void dispenseGlyph() {
        state = GlyphIntakeState.OUTAKE;
    }

    //This method turns off the intake
    public void turnOff() {
        state = GlyphIntakeState.NOTHING;
    }

    //this method is called to stop the thread
    public void stop(){
        servo1.setPower(0);
        servo2.setPower(0);
        running = false;
    }

    public GlyphIntakeState getState(){
        return state;
    }
}