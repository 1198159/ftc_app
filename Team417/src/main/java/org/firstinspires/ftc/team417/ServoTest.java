package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="ServoTests", group = "Swerve")
public class ServoTest extends LinearOpMode
{
    Servo servoPush;

    public void runOpMode()
    {
        servoPush = hardwareMap.servo.get("servoPush");

        servoPush.setPosition(0.5);
        sleep(2000);
        servoPush.setPosition(0.1);
        waitForStart();

        while (opModeIsActive())
        {
            servoPush.setPosition(Range.scale(gamepad1.right_stick_y, -1, 1, 0, 1));
            //servoPush.setPosition(gamepad1.right_stick_y);


            servoPush.setPosition(0.1);
            sleep(4000);
            servoPush.setPosition(0.25);
            sleep(4000);
            servoPush.setPosition(0.5);
            sleep(4000);
            servoPush.setPosition(0.75);
            sleep(4000);
            servoPush.setPosition(0.9);
            sleep(7000);

        }
    }
}

