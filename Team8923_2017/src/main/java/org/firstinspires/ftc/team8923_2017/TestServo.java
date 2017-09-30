package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.SerialNumber;

/**
 * Testing platform for GeeGee servos in prototype 1
 */

@TeleOp(name = "Servo Tester", group = "Test")
public class TestServo extends LinearOpMode
{
    Servo servoL;
    Servo servoR;
    Servo currentServo;
    String currentServoID = "Left Servo";
    double currentServoPos  = 0.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initServos();

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.dpad_right)
            {
                currentServo = servoL;
                currentServoID = "Left Servo";
            }
            else if(gamepad1.dpad_left)
            {
                currentServo = servoR;
                currentServoID = "Right Servo";
            }

            if (gamepad1.dpad_up)
            {
                currentServo.setPosition(currentServo.getPosition() + 0.1);
            }
            else if(gamepad1.dpad_down)
            {
                currentServo.setPosition(currentServo.getPosition() - 0.1);
            }

            currentServoPos = currentServo.getPosition();

            while (!buttonsAreReleased(gamepad1))
            {
                doTelemetry();
                idle();
            }
        }
    }

    public void initServos()
    {
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");
        currentServo = servoL;
    }

    void doTelemetry()
    {
        telemetry.addData("Current Servo", currentServoID);
        telemetry.addData("Current Servo Position", currentServoPos);
        telemetry.update();
    }

    boolean buttonsAreReleased(Gamepad pad)
    {
        return !(pad.a || pad.b || pad.x || pad.y || pad.left_bumper || pad.right_bumper
                || pad.dpad_up || pad.dpad_down || pad.dpad_left || pad.dpad_right
                || pad.left_stick_button || pad.right_stick_button
                || pad.start || pad.back || pad.guide || pad.left_trigger > 0.35
                || pad.right_trigger > 0.35);
    }
}
