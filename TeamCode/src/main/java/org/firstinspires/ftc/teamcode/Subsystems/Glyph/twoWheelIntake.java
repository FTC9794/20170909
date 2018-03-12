package org.firstinspires.ftc.teamcode.Subsystems.Glyph;

import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by ishaa on 1/2/2018.
 */

public class twoWheelIntake implements IGlyph {
    //Create constants
    private double intakePower = 1;
    private double outtakePower = -1;
    //Intake hardware
    private CRServo left, right;

    /**
     * Constructor for a two wheel intake
     * @param leftIntake left wheel on the intake mechanism
     * @param rightIntake right wheel on the intake mechanism
     */
    public twoWheelIntake(CRServo leftIntake, CRServo rightIntake){
        left = leftIntake;
        right = rightIntake;
    }

    /**
     * Constructor for a two wheel intake
     * @param leftIntake left wheel on the intake mechanism
     * @param rightIntake right wheel on the intake mechanism
     * @param intakePower power to set wheels when intaking
     * @param outtakePower power to set wheels when outtaking
     */
    public twoWheelIntake(CRServo leftIntake, CRServo rightIntake, double intakePower, double outtakePower){
        left = leftIntake;
        right = rightIntake;
        this.intakePower = intakePower;
        this.outtakePower = outtakePower;
    }

    /**
     * Set the power of the wheels to being a glyph into the robot's possession
     */
    @Override
    public void secureGlyph() {
        left.setPower(intakePower);
        right.setPower(intakePower);
    }

    /**
     * Set the power of the wheels to dispense a glyph from the robot's possession
     */
    @Override
    public void dispenseGlyph() {
        left.setPower(outtakePower);
        right.setPower(outtakePower);

    }

    /**
     * Changes the height of the glyph lift
     * @param power power to set the motor, ranging from -1.0 to 1.0
     * @param condition the condition that the lift changes position
     * @return true, once the action is completed
     */
    public boolean changeHeight(double power, boolean condition) {
        return false;
    }

    /**
     * Sets the wheel speeds to zero, turning off the mechanism
     */
    @Override
    public void turnOff() {
        left.setPower(0);
        right.setPower(0);
    }
}
