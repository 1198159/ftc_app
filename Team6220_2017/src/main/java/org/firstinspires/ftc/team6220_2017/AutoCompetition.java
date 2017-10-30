package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoCompetition", group = "Autonomous")
public class AutoCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        waitForStart();

        vuforiaHelper.getVumark();

        if (vuforiaHelper.isVisible())
        {
            boolean isLeftBlue = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("leftColor ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("RightColor ", vuforiaHelper.avgRightJewelColor);
        }
        else
            telemetry.addData("vuMark: ", "not visible");

        telemetry.update();
    }

    //todo modify for jewels rather than beacons
    //we use this function to determine the color of jewels and knock them
    public void knockJewel (boolean isLeftBlue, boolean isBlueSide) throws InterruptedException
    {
        if(isBlueSide)
        {
           if(isLeftBlue)
           {

           }
            else
           {

           }
        }
        else
        {
            if(isLeftBlue)
            {

            }
            else
            {

            }
        }
    }
}
