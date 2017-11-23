package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Cole Welch on 10/1/2017.
 */

@Autonomous(name = "Vuforia Test", group = "Autonomous")
public class VuforiaTestBed extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        VuforiaHelper.BlueJewel blueJewel = VuforiaHelper.BlueJewel.UNDETERMINED;

        vuforiaHelper = new VuforiaHelper();
        vuforiaHelper.setupVuforia();

        vuforiaHelper.getVumark();

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

        waitForStart();
    }
}
