package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Sarthak on 9/24/2017.
 */

public interface IDrivetrain {

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
    boolean moveIMU(double currentPosition, double targetPosition, double rampDownTargetPosition, double rampUpTargetPosition, double maxPower, double lowPower, double moveAngle, double[] PIDGain, double endOrientationAngle, double allowableDistanceError);

    /**
     *
     * @param desiredAngle The angle to which to pivot to
     * @param rampDownAngle The angle at which to start slowing down
     * @param maxPower The max power to pivot at
     * @param minPower The min power to pivot at
     * @param correctionTime The amount of time to spend correcting to stay within the desired range
     * @return
     */
    boolean pivotIMU(double desiredAngle, double rampDownAngle, double maxPower, double minPower, double correctionTime);


    //boolean moveNoIMU(double currentPosition, double targetPosition, double rampDownTargetPosition, double rampUpTargetPosition, double maxPower, double lowPower, double moveAngle, doudouble angle, double speed, boolean condition, double pivotAmount);

    //boolean balance(double highPower, double lowPower, double moveAngle, double oGain, boolean endCondition);

    //resets the encoders.
    void softResetEncoder();

    // Change the mode of the motors to stop and reset encoders
    void resetEncoders();

    //Averages the motor encoders
    double averageEncoders();

    //Gets the distance traveled using encoders
    double getEncoderDistance();

    //Stops all the motors on the drivetrain
    void stop();

}
