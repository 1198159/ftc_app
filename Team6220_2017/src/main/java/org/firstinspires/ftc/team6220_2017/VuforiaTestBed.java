package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 *  For testing jewel identification without robot hardware
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

        double MM_PER_ANDYMARK_TICK = (Math.PI * Constants.WHEEL_DIAMETER_MM) / (Constants.ANDYMARK_TICKS_PER_ROTATION * Constants.GEAR_RATIO);

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
