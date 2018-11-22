package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Practice_Auto", group="Pushbot")
//@Disabled
public class Practice_Autonomas extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePractice robot           = new HardwarePractice();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
    @Override
    public void runOpMode() {
        double left;
        double leftY;
        double leftX;
        double rightX;
        double armMotorPos;
        double holdPos;
        boolean isHolding = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


//        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Andymark 1120 per revolution.
            robot.armDrive.setPower(0.5);

            sleep(1700);

            robot.armDrive.setPower(0);
            robot.leftBackDrive.setPower(-0.7);
            robot.leftFrontDrive.setPower(0.7);
            robot.rightBackDrive.setPower(0.7);
            robot.rightFrontDrive.setPower(-0.7);

            sleep(1300);

            robot.leftBackDrive.setPower(0.7);
            robot.leftFrontDrive.setPower(0.7);
            robot.rightBackDrive.setPower(-0.7);
            robot.rightFrontDrive.setPower(-0.7);

            sleep(400);

            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);

            robot.leftBackDrive.setPower(0.5);
            robot.leftFrontDrive.setPower(0.5);
            robot.rightBackDrive.setPower(0.5);
            robot.rightFrontDrive.setPower(0.5);

            sleep(2300);

            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);

            robot.leftBackDrive.setPower(-1);
            robot.leftFrontDrive.setPower(-1);
            robot.rightBackDrive.setPower(-1);
            robot.rightFrontDrive.setPower(-1);
            //hi ron i ruined the code aaaghghgh ERROR ERROR ERROR!!!

            sleep(200);

            robot.leftBackDrive.setPower(-0.5);
            robot.leftFrontDrive.setPower(-0.5);
            robot.rightBackDrive.setPower(0.5);
            robot.rightFrontDrive.setPower(0.5);

            sleep(1000);

            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);

            robot.leftBackDrive.setPower(0.5);
            robot.leftFrontDrive.setPower(0.5);
            robot.rightBackDrive.setPower(0.5);
            robot.rightFrontDrive.setPower(0.5);

            sleep(700);

            robot.leftBackDrive.setPower(-0.5);
            robot.leftFrontDrive.setPower(-0.5);
            robot.rightBackDrive.setPower(0.5);
            robot.rightFrontDrive.setPower(0.5);

            sleep(650);
            robot.leftBackDrive.setPower(0);
            robot.leftFrontDrive.setPower(0);
            robot.rightBackDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);

            robot.leftBackDrive.setPower(0.5);
            robot.leftFrontDrive.setPower(0.5);
            robot.rightBackDrive.setPower(0.5);
            robot.rightFrontDrive.setPower(0.5);

            sleep(2200);

            while (opModeIsActive()) {
                robot.leftBackDrive.setPower(0);
                robot.leftFrontDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
            }
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            // Send telemetry message to signify robot running;
            telemetry.addData("Is Holding: ", isHolding);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            //sleep(50);
        }
    }
}
