package org.firstinspires.ftc.teamcode.LunaCode;

/**
 * Created by Raashi Dewan on 1/28/2018.
 */
        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.Range;

        import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@TeleOp(name="Luna Teleop", group="Pushbot")
public class LunaTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    LunaHardware robot = new LunaHardware();
    // could also use HardwarePushbotMatrix class.
//    double          clawOffset      = 0;                       // Servo mid position
//    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

//    public static final double MID_SERVO       =  0.5 ;
//
//    public DcMotor  leftMotor   = null;
//    public DcMotor  rightMotor  = null;
//    public DcMotor  armMotor    = null;
//    public Servo leftClaw    = null;
//    public Servo    rightClaw   = null;


    //    public void init(hardwareMap){
//        leftMotor = hardwareMap.dcMotor.get("left_drive");
//        rightMotor = hardwareMap.dcMotor.get("right_drive");
//        armMotor = hardwareMap.dcMotor.get("left_arm");
//
//        //check if motors are AndyMark, if not reverse directions
//        leftMotor.setDirection(DcMotor.Direction.REVERSE);
//        rightMotor.setDirection(DcMotor.Direction.FORWARD);
//
//        // Set all motors to zero power
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
//        armMotor.setPower(0);
//
//        // Set all motors to run without encoders.
//        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        // Define and initialize ALL installed servos.
//        leftClaw = hardwareMap.servo.get("leftClaw");
//        rightClaw = hardwareMap.servo.get("rightClaw");
//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);
//        //Initialize the drive system variables.
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Ready to run");    //
//        telemetry.update();
//    }
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            //move forward
            if(gamepad1.right_stick_y > .1 || gamepad1.left_stick_y > .1){
                robot.rightMotor.setPower(-0.25);
                robot.leftMotor.setPower(-0.25);
            }
            //move backward
            else if(gamepad1.right_stick_y < -.1 || gamepad1.left_stick_y < -.1){
                robot.rightMotor.setPower(0.25);
                robot.leftMotor.setPower(0.25);
            }
            //turn right
            else if(gamepad1.left_stick_x > .1 || gamepad1.right_stick_x > .1){
                robot.leftMotor.setPower(0.25);
                robot.rightMotor.setPower(-0.25);
            }
            //turn left
            else if(gamepad1.left_stick_x < -.1 || gamepad1.right_stick_x < -.1){
                robot.leftMotor.setPower(-0.25);
                robot.rightMotor.setPower(0.25);
            }
            //stop
            else{
                robot.leftMotor.setPower(0);
                robot.rightMotor.setPower(0);
            }


            // Use gamepad left & right Bumpers to open and close the claw
//            if (gamepad1.right_bumper)
//                clawOffset += CLAW_SPEED;
//            else if (gamepad1.left_bumper)
//                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
//            clawOffset = Range.clip(clawOffset, -0.5, 0.5);

           //open
            if(gamepad1.right_bumper == true) {
                robot.Claw.setPosition(0.2);
            }

            //closed
            if(gamepad1.left_bumper == true) {
                robot.Claw.setPosition(1);
            }

            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad1.right_trigger > 0.1)
                robot.armMotor.setPower(robot.ARM_UP_POWER);
            else if (gamepad1.left_trigger > 0.1)
                robot.armMotor.setPower(robot.ARM_DOWN_POWER);
            else
                robot.armMotor.setPower(0.0);

            // Send telemetry message to signify robot running;
//            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
//            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
