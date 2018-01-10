package org.firstinspires.ftc.teamcode.SampleTestCode;

/**
 * Created by Raashi Dewan on 10/13/2017.
 */
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
@Autonomous(name = "IMU Test", group = "Sensor")

public class imuTest2 extends LinearOpMode{
        //Create IMU Object
        BNO055IMU imu;
        //Orientation Object
        Orientation o;
        //Variables to store angle values
        double x, y, z;
        @Override
        public void runOpMode() throws InterruptedException {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            //parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            ElapsedTime runtime = new ElapsedTime();
            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            composeTelemetry();
            telemetry.update();

            // Wait until we're told to go
            waitForStart();
            // Start the logging of measured acceleration
            imu.startAccelerationIntegration(new Position(), new Velocity(), 20);

            // Loop and update the dashboard
            while (opModeIsActive()) {
                o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                x = o.firstAngle;
                y = o.secondAngle;
                z = o.thirdAngle;
                telemetry.addData("Angle on X Axis:", x);//tilt
                telemetry.addData("Angle on Y Axis:", y);//lateral rotations
                telemetry.addData("Angle on Z Axis:", z);//up and dows
                telemetry.addData("X Velocity:", imu.getVelocity().xVeloc);
                telemetry.addData("Y: ", imu.getVelocity().yVeloc);
                telemetry.addData("Z: ", imu.getVelocity().zVeloc);
                telemetry.addData("Position:", Math.sqrt((Math.pow(imu.getPosition().x, 2.0) +
                        Math.pow(imu.getPosition().y, 2.0) + Math.pow(imu.getPosition().z, 2.0))));
                telemetry.update();
            }

        }
        void composeTelemetry() {

            // At the beginning of each telemetry update, grab a bunch of data
            // from the IMU that we will then display in separate lines.
            telemetry.addLine()
                    .addData("Status: ", new Func<String>() {
                        @Override public String value() {
                            return imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("Calibration Status: ", new Func<String>() {
                        @Override public String value() {
                            return imu.getCalibrationStatus().toString();
                        }
                    });
        }
}
