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
        initializeAuto();

        vuforiaHelper.getVumark();

        //if the vuMark is not visible, vuforia will tell us
        if (vuforiaHelper.isVisible())
        {
            boolean isLeftBlue = vuforiaHelper.getLeftJewelColor();
            boolean isBlueSide = true;
            telemetry.addData("leftColor ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("RightColor ", vuforiaHelper.avgRightJewelColor);
            knockJewel(isLeftBlue,isBlueSide);
        }
        else
            telemetry.addData("vuMark: ", "not visible");

        telemetry.update();

        waitForStart();
    }

    //todo modify for jewels rather than beacons
    //we use this function to determine the color of jewels and knock them
    public void knockJewel (boolean isLeftBlue, boolean isBlueSide) throws InterruptedException
    {
        jewelJostlerServo.setPosition(Constants.JEWEL_JOSTLER_DEPLOYED);

        if(isBlueSide)
        {
            if(isLeftBlue)
            {
                turnTo(-30);
                pauseWhileUpdating(500);
                turnTo(0);
            }
            else
            {
                turnTo(30);
                pauseWhileUpdating(500);
                turnTo(0);
            }
        }
        else
        {
            if(isLeftBlue)
            {
                turnTo(30);
                pauseWhileUpdating(500);
                turnTo(0);
            }
            else
            {
                turnTo(-30);
                pauseWhileUpdating(500);
                turnTo(0);
            }
        }
    }
}
