package org.firstinspires.ftc.teamcode;

/**
 * Created by Sarthak on 9/24/2017.
 */

public interface IDrivetrain {

    /**
     *
     * @param highPower The maximum speed the robot will move at
     * @param lowPower The lowest speed the robot will move at
     * @param powerChange The distance or time the robot is away from the target
     * @param powerGain How much the power change affects the speed (speed = powerChange*powerGain but has to be inbetween low and high power)
     * @param moveAngle The angle at which the robot will move in the frame of reference of the driver
     * @param oGain How much
     * @param pGain
     * @param endOrientationAngle
     * @param endCondition
     * @param timeAfterAngle
     * @return
     */
    boolean move(double highPower, double lowPower, double powerChange, double powerGain, double moveAngle, double oGain, double pGain, double endOrientationAngle, boolean endCondition, double timeAfterAngle);

    /**
     * balances the robot on the balancing stone
     * @param highPower The maximum speed the robot will move at
     * @param lowPower The lowest speed the robot will move at
     * @param moveAngle the angle at which the robot maintain on the balancing board
     * @param oGain gain to maintain angle
     * @param endCondition when the robot will stop balancing
     * @return true if still balancing, false if finished balancing
     */
    boolean balance(double highPower, double lowPower, double moveAngle, double oGain, boolean endCondition);

    //resets the encoders without stopping the robot.
    void softResetEncoder();

    // Resets encoders and stops the motors
    void resetEncoders();

    //Averages the motor encoders
    double averageEncoders();

    //Stops all the motors on the robot
    void stop();

    void setPowerZero();
}
