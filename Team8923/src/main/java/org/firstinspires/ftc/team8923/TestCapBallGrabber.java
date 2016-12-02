package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Cap Ball Grabber Test", group = "Tests")
public class TestCapBallGrabber extends LinearOpMode
{
    private Servo servoLeft;
    private Servo servoRight;

    private double servoPosition = 0.5;

    private static final double DELTA_POSITION = 0.05;

    private boolean wasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        servoRight = hardwareMap.servo.get("servoRight");
        servoLeft = hardwareMap.servo.get("servoLeft");

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.dpad_up && !wasPressed)
            {
                servoPosition += DELTA_POSITION;
                wasPressed = true;
            }
            else if(gamepad1.dpad_down && !wasPressed)
            {
                servoPosition -= DELTA_POSITION;
                wasPressed = true;
            }
            else if(!gamepad1.dpad_down && !gamepad1.dpad_up)
                wasPressed = false;

            servoRight.setPosition(servoPosition);
            servoLeft.setPosition(servoPosition);

            // Inform drivers
            telemetry.addData("Position", servoPosition);

            telemetry.update();
            idle();
        }
    }
}
