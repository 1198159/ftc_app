package org.firstinspires.ftc.team8923;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Used to test and debug detection of beacon colors
 */
@TeleOp(name = "Beacon Test", group = "Tests")
public class TestBeaconColor extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initAuto();

        waitForStart();

        servoBeaconPusherDeploy.setPosition(ServoPositions.BEACON_EXTEND.pos);

        telemetry.setMsTransmissionInterval(30);

        while(opModeIsActive())
        {
            // RGB values and hue from left sensor
            int leftRed = colorSensorLeft.red();
            int leftGreen = colorSensorLeft.green();
            int leftBlue = colorSensorLeft.blue();
            float leftHue = getHueFromSensor(colorSensorLeft);
            telemetry.addData("Left Red", leftRed);
            telemetry.addData("Left Green", leftGreen);
            telemetry.addData("Left Blue", leftBlue);
            telemetry.addData("Left Hue: ", leftHue);

            // Add space between left and right sensor so it's easier to read
            telemetry.addData("", "");

            // RGB values and hue from right sensor
            int rightRed = colorSensorRight.red();
            int rightGreen = colorSensorRight.green();
            int rightBlue = colorSensorRight.blue();
            float rightHue = getHueFromSensor(colorSensorRight);
            telemetry.addData("Right Red", rightRed);
            telemetry.addData("Right Green", rightGreen);
            telemetry.addData("Right Blue", rightBlue);
            telemetry.addData("Right Hue: ", rightHue);

            telemetry.addData("", "");
            telemetry.addData("Timer", getRuntime());

            telemetry.update();
            idle();
        }
    }
}
