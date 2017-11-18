package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// todo Finish implementing setup routine created in MasterAutonomous
@Autonomous(name = "AutoCompetition", group = "Autonomous")

public class AutoCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        waitForStart();

        vuforiaHelper.getVumark();


        // Get the next objective from the routine, and run it
        /*switch(alliance)
        {
            // Only complete the requested objective
            case BLUE:
                isBlueSide = true;
                break;
            case RED:
                isBlueSide = false;
                break;
        }*/


        //if the vuMark is not visible, vuforia will tell us
        if (vuforiaHelper.isVisible())
        {
            boolean isLeftBlue = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("leftColor ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("RightColor ", vuforiaHelper.avgRightJewelColor);
            knockJewel(isLeftBlue,isBlueSide);
        }
        else
            telemetry.addData("vuMark: ", "not visible");

        telemetry.update();
    }
}
