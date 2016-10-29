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
        waitForStart();

        vuforiaLocator.startTracking();

        while(opModeIsActive())
        {
            // Get color values of both sides of beacon
            int colorLeft = vuforiaLocator.getPixelColor(-60, 230, 30);
            int colorRight = vuforiaLocator.getPixelColor(60, 230, 30);

            telemetry.addData("LeftRed", Color.red(colorLeft));
            telemetry.addData("LeftGreen", Color.green(colorLeft));
            telemetry.addData("LeftBlue", Color.blue(colorLeft));
            telemetry.addData("", "");
            telemetry.addData("RightRed", Color.red(colorRight));
            telemetry.addData("RightGreen", Color.green(colorRight));
            telemetry.addData("RightBlue", Color.blue(colorRight));

            telemetry.update();
            idle();
        }
    }
}
