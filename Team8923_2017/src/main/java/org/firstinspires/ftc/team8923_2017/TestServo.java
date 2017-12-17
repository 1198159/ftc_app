package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.SerialNumber;

/**
 * Testing platform for GeeGee servos in prototype 1
 */

@Disabled
@TeleOp(name = "Servo Tester", group = "Test")
public class TestServo extends LinearOpMode
{
    Servo servoL;
    Servo servoR;
    char currentServoID = 'L';
    double currentServoPosL  = 0.0;
    double currentServoPosR  = 0.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initServos();

        waitForStart();

        servoL.setPosition(0.5);
        servoR.setPosition(0.0);

        while(opModeIsActive())
        {
            if(gamepad1.dpad_left)
            {
                currentServoID = 'L';
            }
            else if(gamepad1.dpad_right)
            {
                currentServoID = 'R';
            }

            if (gamepad1.dpad_up)
            {
                if(currentServoID == 'R')
                    servoR.setPosition(currentServoPosR += 0.1);
                else if(currentServoID == 'L')
                    servoL.setPosition(currentServoPosL += 0.1);
            }
            else if (gamepad1.dpad_down)
            {
                if (currentServoID == 'R')
                    servoR.setPosition(currentServoPosR -= 0.1);
                else if (currentServoID == 'L')
                    servoL.setPosition(currentServoPosL -= 0.1);
            }

            doTelemetry();

            while (!buttonsAreReleased(gamepad1)){}

        }
    }

    public void initServos()
    {
        servoL = hardwareMap.get(Servo.class, "servoL");
        servoR = hardwareMap.get(Servo.class, "servoR");
    }

    void doTelemetry()
    {
        telemetry.addData("Current Servo", currentServoID);
        if(currentServoID == 'L')
            telemetry.addData("Current Servo Position", currentServoPosL);
        else if(currentServoID == 'R')
            telemetry.addData("Current Servo Position", currentServoPosR);
        if(gamepad1.dpad_up)
            telemetry.addLine("Dpad_Up");
        else if(gamepad1.dpad_down)
            telemetry.addLine("Dpad_Down");
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
