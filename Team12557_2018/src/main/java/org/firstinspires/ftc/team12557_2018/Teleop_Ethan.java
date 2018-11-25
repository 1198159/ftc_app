package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Ethan Teleop", group="Pushbot")
//@Disabled
public class Teleop_Ethan extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePractice robot           = new HardwarePractice();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        double drive;  //
        double strafe;
        double rotation;
        double armMotorPos; //for debug only

        // Initialize the hardware variables.
        robot.init(hardwareMap);

        robot.armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Andymark 1120 per revolution.
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            rotation = gamepad1.right_stick_x;
            
            //Mecanum drive
            mecanumDrive(drive, strafe, rotation);

            //when right bumper is pressed/hold, arm goes up until it is released
            if (gamepad1.right_bumper) {
                robot.armDrive.setPower(1);  //make the movement smooth
                if (robot.armDrive.getCurrentPosition() >12450) {
                    robot.armDrive.setPower(0);
                }
            } else {
                robot.armDrive.setPower(0);
            }

            //when left bumper is pressed/hold, arm goes down until it is release
            if (gamepad1.left_bumper) {
                robot.armDrive.setPower(-1);
                if (robot.armDrive.getCurrentPosition() < -20) {
                    robot.armDrive.setPower(0);
                }
            } else {
                robot.armDrive.setPower(0);
            }
            
             // Send telemetry message to signify robot running;
            armMotorPos = robot.armDrive.getCurrentPosition();
            telemetry.addData("left stick gamepad 1 X: ",  "%.2f", strafe );
            telemetry.addData("left stick gamepad 1 Y: ", "%.2f", drive);
            telemetry.addData("right stick gamepad 1 X: ", "%.2f", rotation);
            telemetry.addData("Arm Motor position: ", "%.2f", armMotorPos);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            //sleep(50);
        }
    }

    protected void mecanumDrive ( double d, double s, double r)
    {
        double wheelPowers[] = new double[4];

        wheelPowers[0] = d + s + r;
        wheelPowers[1] = d - s - r;
        wheelPowers[2] = d -s + r;
        wheelPowers[3] = d + s - r;

        normalize(wheelPowers);

        robot.leftFrontDrive.setPower(wheelPowers[0]);
        robot.rightFrontDrive.setPower(wheelPowers[1]);
        robot.leftBackDrive.setPower(wheelPowers[2]);
        robot.rightBackDrive.setPower(wheelPowers[3]);
    }   //Mecanum Drive

    public void normalize ( double[] wheelSpeeds)
    {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double magnitude = Math.abs(wheelSpeeds[i]);
            if (magnitude > maxMagnitude) {
                maxMagnitude = magnitude;
            }
        }

        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }
    }   //normalizeInPlace
}
