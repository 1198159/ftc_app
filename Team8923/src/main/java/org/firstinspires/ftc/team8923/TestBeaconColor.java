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

        vuforiaLocator.startTracking();

        servoBeaconPusherDeploy.setPosition(ServoPositions.BEACON_EXTEND.pos);

        while(opModeIsActive())
        {
            // Get color values of both sides of beacon
            int colorLeft = vuforiaLocator.getPixelColor(-40, 170, 30);
            int colorRight = vuforiaLocator.getPixelColor(40, 170, 30);

            float[] leftHSV = new float[3];
            float[] rightHSV = new float[3];

            Color.colorToHSV(colorLeft, leftHSV);
            Color.colorToHSV(colorRight, rightHSV);

            telemetry.addData("Hue Left", leftHSV[0]);
            telemetry.addData("Hue Right", rightHSV[0]);
            telemetry.addData("", "");
            telemetry.addData("LeftRed", Color.red(colorLeft));
            telemetry.addData("LeftGreen", Color.green(colorLeft));
            telemetry.addData("LeftBlue", Color.blue(colorLeft));
            telemetry.addData("", "");
            telemetry.addData("RightRed", Color.red(colorRight));
            telemetry.addData("RightGreen", Color.green(colorRight));
            telemetry.addData("RightBlue", Color.blue(colorRight));
            telemetry.addData("", "");

            // Color sensor data
            int colorSensorColor = colorSensorLeft.argb();
            float[] colorSensorHSV = new float[3];

            int red = colorSensorLeft.red();
            int green = colorSensorLeft.green();
            int blue = colorSensorLeft.blue();

            double scalar = Math.max(red, Math.max(green, blue));

            // 255 is max rgb value
            red *= 255.0 / scalar;
            green *= 255.0 / scalar;
            blue *= 255.0 / scalar;

            int argb = Color.argb(0, red, green, blue);
            Color.colorToHSV(argb, colorSensorHSV);

            telemetry.addData("Color Sensor Hue", colorSensorHSV[0]);
            telemetry.addData("Color Sensor Red", red);
            telemetry.addData("Color Sensor Green", green);
            telemetry.addData("Color Sensor Blue", blue);

            telemetry.update();
            idle();
        }
    }
}
