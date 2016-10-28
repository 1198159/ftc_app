package org.firstinspires.ftc.team8923;

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
            // Get color from just above image target
            telemetry.addData("Color", vuforiaLocator.getPixelColor(0, 100, 0));

            telemetry.update();
            idle();
        }
    }
}
