package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Mridula on 10/29/2017.
 */
@Autonomous(name = "Auto Blue", group = "Autonomous")

public class AutoBlue extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        waitForStart();

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
    }

    //todo modify for jewels rather than beacons
    //we use this function to determine the color of jewels and knock them
    public void knockJewel (boolean isLeftBlue, boolean isBlueSide) throws InterruptedException
    {
        golfClubServo.setPosition(0.15);

        if(isBlueSide)
        {
            if(isLeftBlue)
            {
                turnTo(-30);
            }
            else
            {
                turnTo(30);
            }
        }
        else
        {
            if(isLeftBlue)
            {
                turnTo(30);
            }
            else
            {
                turnTo(-30);
            }
        }
    }
}
