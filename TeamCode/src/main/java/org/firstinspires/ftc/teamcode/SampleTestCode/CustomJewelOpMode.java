package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CustomJewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by ishaa on 1/28/2018.
 */

public class CustomJewelOpMode extends OpMode {
    CustomJewelDetector detector;
    @Override
    public void init() {
        detector = new CustomJewelDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.enable();
    }

    @Override
    public void loop() {
        telemetry.addData("jewel", detector.getColor());
    }
}
