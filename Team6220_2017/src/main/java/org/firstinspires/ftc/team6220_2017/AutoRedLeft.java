package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Left", group = "Autonomous")

public class AutoRedLeft extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();
        waitForStart();

        //setRobotStartingOrientation(0);

        boolean isBlueSide = false;
        VuforiaHelper.BlueJewel blueJewel = VuforiaHelper.BlueJewel.UNDETERMINED;


        vuforiaHelper.getVumark();


        // if the vuMark is not visible, vuforia will tell us
        if (vuforiaHelper.isVisible())
        {
            blueJewel = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("Left Hue: ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("Right Hue: ", vuforiaHelper.avgRightJewelColor);

            knockJewel(blueJewel, isBlueSide);
        }
        else
        {
            telemetry.addData("vuMark: ", "not visible");
        }

        telemetry.update();
    }
}
