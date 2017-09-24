package org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

/**
 * Created by ishaa on 6/30/2017.
 */

public class AcceleratedDcMotor implements DcMotor {
    private DcMotor motor;
    private double targetPower;
    private double currentPower;
    private double rateOfAccDec;

    public AcceleratedDcMotor(DcMotor motor, double rateOfAccDec){
        this.motor = motor;
        this.rateOfAccDec = rateOfAccDec;
        targetPower = 0;
        currentPower = 0;
    }
    @Override
    public MotorConfigurationType getMotorType() {
        return motor.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        motor.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return motor.getController();
    }

    @Override
    public int getPortNumber() {
        return motor.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior();
    }

    @Override
    @Deprecated
    public void setPowerFloat() {
        motor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return motor.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        motor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return motor.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        motor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return motor.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        motor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection();
    }

    @Override
    public void setPower(double power) {
        targetPower = power;
    }

    @Override
    public double getPower() {
        return currentPower;
    }

    @Override
    public Manufacturer getManufacturer() {
        return motor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        motor.close();
    }

    public synchronized void update(){
        if(targetPower<currentPower){
            currentPower-=rateOfAccDec;
            currentPower = Math.max(currentPower, targetPower);
            motor.setPower(currentPower);
        }else if(targetPower>currentPower){
            currentPower+=rateOfAccDec;
            currentPower = Math.min(currentPower, targetPower);
            motor.setPower(currentPower);
        }
    }
    public synchronized void updateRawPower(){
        motor.setPower(targetPower);
        currentPower = targetPower;
    }

    public synchronized void stopMotor(){
        this.setPower(0);
        updateRawPower();
    }
}
