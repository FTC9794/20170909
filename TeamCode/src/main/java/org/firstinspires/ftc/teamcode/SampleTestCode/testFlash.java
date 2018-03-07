package org.firstinspires.ftc.teamcode.SampleTestCode;

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * Created by ishaa on 3/6/2018.
 */

public class testFlash extends LinearOpMode {
    Camera camera;
    @Override
    public void runOpMode() throws InterruptedException {
        //ToDo something
        camera = Camera.open();
        Camera.Parameters parameters = camera.getParameters();
        parameters.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
        camera.setParameters(parameters);
        camera.startPreview();
        waitForStart();
        camera = Camera.open();
        parameters = camera.getParameters();
        parameters.setFlashMode(Camera.Parameters.FLASH_MODE_OFF);
        camera.setParameters(parameters);
        camera.stopPreview();
        camera.release();
    }
}
