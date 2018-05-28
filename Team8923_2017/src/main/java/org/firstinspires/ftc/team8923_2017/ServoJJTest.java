package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Testing platform for GeeGee servos in prototype 1
 */

@TeleOp(name = "ServoJJ Tester", group = "Test")
@Disabled
public class ServoJJTest extends LinearOpMode
{
    Servo servoJJ = null;
    double servoPos = 0.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initServos();
        waitForStart();

        servoJJ.setPosition(0.0);
        sleep(3000);
        servoJJ.setPosition(0.5);
        sleep(3000);
        servoJJ.setPosition(0.69);
        sleep(3000);
        servoJJ.setPosition(0.5);
        //doTelemetry();
    }
    public void initServos()
    {
        servoJJ = hardwareMap.get(Servo.class, "servoJJ");
    }

    void doTelemetry()
    {
        telemetry.addData("Servo Pos", servoPos);
        telemetry.update();
    }
}
