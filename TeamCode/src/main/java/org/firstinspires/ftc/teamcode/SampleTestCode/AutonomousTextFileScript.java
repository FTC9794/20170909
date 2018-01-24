package org.firstinspires.ftc.teamcode.SampleTestCode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.apache.commons.jexl3.JexlBuilder;
import org.apache.commons.jexl3.JexlContext;
import org.apache.commons.jexl3.JexlEngine;
import org.apache.commons.jexl3.JexlExpression;
import org.apache.commons.jexl3.MapContext;
import org.apache.commons.jexl3.internal.introspection.EnumerationIterator;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Rule;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.IColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor.LynxColorRangeSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.AcceleratedDcMotor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.IDrivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain.OmniDirectionalDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.DualWheelIntake;
import org.firstinspires.ftc.teamcode.Subsystems.Glyph.FourArmRotatingGlyph;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.BoschIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.IIMU;
import org.firstinspires.ftc.teamcode.Subsystems.IMU.NavxIMU;
import org.firstinspires.ftc.teamcode.Subsystems.Jewel.TwoPointJewelArm;
import org.firstinspires.ftc.teamcode.Subsystems.Relic.ClawThreePoint;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by Sarthak on 10/22/2017.
 */
@Autonomous(name = "Autonomous Text File Script", group = "Test")
public class AutonomousTextFileScript extends LinearOpMode{

    Servo pan, tilt;
    CRServo rightWheel1, rightWheel2, leftWheel1, leftWheel2;
    Servo spin;
    DcMotor rf, rb, lf, lb;
    DcMotor lift;
    DcMotor relic_extension;
    Servo relic_claw, relic_arm, relic_tilt;
    DigitalChannel glyphLimit;
    BNO055IMU revIMU;
    LynxI2cColorRangeSensor lynx;

    OmniDirectionalDrive drive;
    TwoPointJewelArm jewel;
    DualWheelIntake intake;
    ClawThreePoint relic;
    IIMU imu;
    IColorSensor color;

    List<DcMotor> driveMotors;

    JexlEngine jexl = new JexlBuilder().create();
    String autoFileName = "test.txt";
    File autoFile = AppUtil.getInstance().getSettingsFile(autoFileName);
    String fileText = "";
    String[] inputs;
    String[][] lookupTable;
    int lookupCount = 0;
    final int START_STATE = 0;
    final int STATE_CONDITION = 1;
    final int STATE_ACTION = 2;
    final int END_STATE = 3;

    final double COUNTS_PER_INCH = 45;
    final double ENCODER_OFFSET = 30;


    @Override
    public void runOpMode() throws InterruptedException {
        pan = hardwareMap.servo.get("jewel_pan");
        tilt = hardwareMap.servo.get("jewel_tilt");

        rightWheel1 = hardwareMap.crservo.get("right_glyph1");
        leftWheel1 = hardwareMap.crservo.get("left_glyph1");
        rightWheel2 = hardwareMap.crservo.get("right_glyph2");
        leftWheel2 = hardwareMap.crservo.get("left_glyph2");

        spin = hardwareMap.servo.get("spin_grip");

        rf = hardwareMap.dcMotor.get("right_front");
        rb = hardwareMap.dcMotor.get("right_back");
        lf = hardwareMap.dcMotor.get("left_front");
        lb = hardwareMap.dcMotor.get("left_back");
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors = new ArrayList<>();
        driveMotors.add(rf);
        driveMotors.add(rb);
        driveMotors.add(lf);
        driveMotors.add(lb);
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        lift = hardwareMap.dcMotor.get("glyph_lift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        relic_extension = hardwareMap.dcMotor.get("relic_extension");
        relic_extension.setDirection(DcMotorSimple.Direction.REVERSE);
        relic_extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relic_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relic_extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relic_arm = hardwareMap.servo.get("relic_arm");
        relic_claw = hardwareMap.servo.get("relic_claw");
        relic_tilt = hardwareMap.servo.get("relic_tilt");

        glyphLimit = hardwareMap.digitalChannel.get("glyph_limit");

        revIMU = hardwareMap.get(BNO055IMU.class, "imu");
        lynx = (LynxI2cColorRangeSensor) hardwareMap.get("jewel_color");

        color = new LynxColorRangeSensor(lynx);

        imu = new BoschIMU(revIMU);
        imu.initialize();

        drive = new OmniDirectionalDrive(driveMotors, imu, telemetry);
        jewel = new TwoPointJewelArm(pan, tilt, color, telemetry);
        intake = new DualWheelIntake(rightWheel1, rightWheel2, leftWheel1, leftWheel2, spin, lift, glyphLimit, telemetry);
        relic = new ClawThreePoint(relic_extension, relic_arm, relic_tilt, relic_claw);

        fileText = ReadWriteFile.readFile(autoFile);
        inputs = fileText.split("~");
        lookupTable = new String[100][100];
        for(int i = 0; i<inputs.length; i++){
            String currentLine = inputs[i];
            String[] currentParams = currentLine.split("/");

            for(int j = 0; j< currentParams.length; j++){
                lookupTable[i][j] = currentParams[j];
            }

        }
        boolean hasStart = false;
        for(int i = 0; i < lookupTable.length; i++){
            if(lookupTable[i][START_STATE].trim().toLowerCase().equals("start")){
                hasStart = true;
                break;
            }
        }
        boolean hasEnd = false;
        for(int i = 0; i < lookupTable.length; i++){
            if(lookupTable[i][END_STATE].trim().toLowerCase().equals("end")){
                hasEnd = true;
                break;
            }
        }
        telemetry.addData("Init", "File read, data stored");
        telemetry.addData("Has Start State", hasStart);
        telemetry.addData("Has End State", hasEnd);
        telemetry.update();
        String jexlActionString = "";
        JexlExpression jexlActionExp, booleanExp;
        JexlContext jexlActionContext, booleanContext;
        String booleanString = "";
        waitForStart();
        while(opModeIsActive()){
            if(lookupTable[lookupCount][STATE_CONDITION].equals("-")){
                jexlActionString = lookupTable[lookupCount][STATE_ACTION];
                jexlActionExp = jexl.createExpression(jexlActionString);
                jexlActionContext = new MapContext();
                if(jexlActionString.indexOf("drive") != -1){
                    jexlActionContext.set("drive", drive);
                }else if(jexlActionString.indexOf("jewel") != -1){
                    jexlActionContext.set("jewel", jewel);
                }else if(jexlActionString.indexOf("intake") != -1){
                    jexlActionContext.set("intake", intake);
                }else if(jexlActionString.indexOf("relic") != -1){
                    jexlActionContext.set("relic", relic);
                }
                Object evaluatedAction = jexlActionExp.evaluate(jexlActionContext);
                while(Boolean.parseBoolean(evaluatedAction.toString())){
                    evaluatedAction = jexlActionExp.evaluate(jexlActionContext);
                }
                lookupCount++;
                drive.resetEncoders();
            }else{
                booleanString = lookupTable[lookupCount][STATE_CONDITION];
                booleanExp = jexl.createExpression(booleanString);
                booleanContext = new MapContext();
                if(booleanString.indexOf("drive") != -1){
                    booleanContext.set("drive", drive);
                }else if(booleanString.indexOf("jewel") != -1){
                    booleanContext.set("jewel", jewel);
                }else if(booleanString.indexOf("intake") != -1){
                    booleanContext.set("intake", intake);
                }else if(booleanString.indexOf("relic") != -1){
                    booleanContext.set("relic", relic);
                }
                Object evaluatedBoolean = booleanExp.evaluate(booleanContext);
                while(Boolean.parseBoolean(evaluatedBoolean.toString())){
                    jexlActionString = lookupTable[lookupCount][STATE_ACTION];
                    jexlActionExp = jexl.createExpression(jexlActionString);
                    jexlActionContext = new MapContext();
                    if(jexlActionString.indexOf("drive") != -1){
                        jexlActionContext.set("drive", drive);
                    }else if(jexlActionString.indexOf("jewel") != -1){
                        jexlActionContext.set("jewel", jewel);
                    }else if(jexlActionString.indexOf("intake") != -1){
                        jexlActionContext.set("intake", intake);
                    }else if(jexlActionString.indexOf("relic") != -1){
                        jexlActionContext.set("relic", relic);
                    }
                    Object evaluatedAction = jexlActionExp.evaluate(jexlActionContext);

                    booleanString = lookupTable[lookupCount][STATE_CONDITION];
                    booleanExp = jexl.createExpression(booleanString);
                    booleanContext = new MapContext();
                    if(booleanString.indexOf("drive") != -1){
                        booleanContext.set("drive", drive);
                    }else if(booleanString.indexOf("jewel") != -1){
                        booleanContext.set("jewel", jewel);
                    }else if(booleanString.indexOf("intake") != -1){
                        booleanContext.set("intake", intake);
                    }else if(booleanString.indexOf("relic") != -1){
                        booleanContext.set("relic", relic);
                    }
                    evaluatedBoolean = booleanExp.evaluate(booleanContext);
                }
                lookupCount++;
                drive.resetEncoders();
            }
        }

    }
}
