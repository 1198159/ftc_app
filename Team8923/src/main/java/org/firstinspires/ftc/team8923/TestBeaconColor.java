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

        while(opModeIsActive())
        {
            // Get color values of both sides of beacon
//            int colorLeft = vuforiaLocator.getPixelColor(-40, 170, 30);
//            int colorRight = vuforiaLocator.getPixelColor(40, 170, 30);
//
//            float[] leftHSV = new float[3];
//            float[] rightHSV = new float[3];
//
//            Color.colorToHSV(colorLeft, leftHSV);
//            Color.colorToHSV(colorRight, rightHSV);
//
//            telemetry.addData("Hue Left", leftHSV[0]);
//            telemetry.addData("Hue Right", rightHSV[0]);
//            telemetry.addData("", "");
//            telemetry.addData("LeftRed", Color.red(colorLeft));
//            telemetry.addData("LeftGreen", Color.green(colorLeft));
//            telemetry.addData("LeftBlue", Color.blue(colorLeft));
//            telemetry.addData("", "");
//            telemetry.addData("RightRed", Color.red(colorRight));
//            telemetry.addData("RightGreen", Color.green(colorRight));
//            telemetry.addData("RightBlue", Color.blue(colorRight));
//            telemetry.addData("", "");
//
//            // Color sensor data
//            int colorSensorColor = colorSensorLeft.argb();
//            float[] colorSensorHSV = new float[3];
//
//            int red = colorSensorLeft.red();
//            int green = colorSensorLeft.green();
//            int blue = colorSensorLeft.blue();
//
//            double scalar = Math.max(red, Math.max(green, blue));
//
//            // 255 is max rgb value
//            red *= 255.0 / scalar;
//            green *= 255.0 / scalar;
//            blue *= 255.0 / scalar;
//
//            int argb = Color.argb(0, red, green, blue);
//            Color.colorToHSV(argb, colorSensorHSV);
//
//            telemetry.addData("Color Sensor Hue", colorSensorHSV[0]);
//            telemetry.addData("Color Sensor Red", red);
//            telemetry.addData("Color Sensor Green", green);
//            telemetry.addData("Color Sensor Blue", blue);

            //float[] colorLeft = new float[3];
            float[] colorRight = new float[3];

            // We don't care about the green value, and it sometimes messes us up, so remove it
            //int argbLeft = colorSensorLeft.argb();
            int argbRight = colorSensorRight.argb();
            int red = colorSensorRight.red();
            int green = colorSensorRight.green();
            int blue = colorSensorRight.blue();
            telemetry.addData("Color Sensor Red", red);
            telemetry.addData("Color Sensor Green", green);
            telemetry.addData("Color Sensor Blue", blue);
            telemetry.addData("Color Sensor Red", Color.red(argbRight));
            telemetry.addData("Color Sensor Green", Color.green(argbRight));
            telemetry.addData("Color Sensor Blue", Color.blue(argbRight));
            //argbLeft = Color.argb(0, Color.red(argbLeft), 0, Color.blue(argbLeft));
            argbRight = Color.argb(0, Color.red(argbRight), 0, Color.blue(argbRight));
            //Color.colorToHSV(argbLeft, colorLeft);
            Color.colorToHSV(argbRight, colorRight);

            // Red value will sometimes be near 0 rather than 360. If so, make it above 360
            // We never get any values near 90 degrees, so anything lower must be red
            //if(colorLeft[0] < 90)
                //colorLeft[0] += 360;
            if(colorRight[0] < 90)
                colorRight[0] += 360;

            //telemetry.addData("Left hue: ", colorLeft[0]);
            telemetry.addData("Right hue: ", colorRight[0]);

            telemetry.update();
            idle();
        }
    }
}
