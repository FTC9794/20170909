package org.firstinspires.ftc.teamcode.SampleTestCode;

/**
 * Created by Raashi Dewan on 10/12/2017.
 */
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
//import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;





@Autonomous(name = "IMU Java", group = "Sensor")
@Disabled
public class imuTest {

    public class IMU_Test extends LinearOpMode
    {
        // The IMU sensor object
        BNO055IMU imu;

        // State used for updating telemetry

        @Override public void runOpMode() {

            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            //parameters.accelerationIntegrationAlgorithm = new ();
            ElapsedTime runtime = new ElapsedTime();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            // Set up our telemetry dashboard
//            composeTelemetry();
            if(imu.getCalibrationStatus().equals(true) && imu.getSystemStatus().equals(true)){
                telemetry.addData("Status", "Press Start");
            }
            telemetry.update();

            // Wait until we're told to go
            waitForStart();

            // Start the logging of measured acceleration
            imu.startAccelerationIntegration(new Position(), new Velocity(), 100);

            // Loop and update the dashboard
            while (opModeIsActive()) {

                telemetry.addData("IMU Velocity: ", imu.getVelocity());
                telemetry.addData("X: ", imu.getVelocity().xVeloc);
                telemetry.addData("Y: ", imu.getVelocity().yVeloc);
                telemetry.addData("Z: ", imu.getVelocity().zVeloc);
                telemetry.addData("Integration Velocity 2: ", imu.getVelocity().toString());
                telemetry.addData("Orientation", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES));
                telemetry.update();

                Math.sqrt((Math.pow(imu.getPosition().x, 2.0) +
                        Math.pow(imu.getPosition().y, 2.0) + Math.pow(imu.getPosition().z, 2.0)));
            }
        }

        //----------------------------------------------------------------------------------------------
        // Telemetry Configuration
        //----------------------------------------------------------------------------------------------

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

        //----------------------------------------------------------------------------------------------
        // Formatting
        //----------------------------------------------------------------------------------------------

        String formatAngle(AngleUnit angleUnit, double angle) {
            return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        String formatDegrees(double degrees){
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }
    }

}
