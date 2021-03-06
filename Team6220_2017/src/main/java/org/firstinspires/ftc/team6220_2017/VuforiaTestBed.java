package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 *  For testing jewel identification without robot hardware.
 */

@Autonomous(name = "Vuforia Test", group = "Autonomous")
//@Disabled
public class VuforiaTestBed extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        vuforiaHelper = new VuforiaHelper();
        vuforiaHelper.setupVuforia();

        vuforiaHelper.getVumark();

        waitForStart();


        //if the vuMark is not visible, vuforia will tell us
        if (vuforiaHelper.isVisible())
        {
            blueJewel = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("Left Hue: ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("Right Hue: ", vuforiaHelper.avgRightJewelColor);
        }
        else
            telemetry.addData("vuMark ", "not visible");

        telemetry.update();
    }
}
