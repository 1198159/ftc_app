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

        runSetup();

        waitForStart();

        vuforiaHelper.getVumark();


        // Score jewel-------------------------------------------------
        // If the vuMark is not visible, vuforia will tell us and the robot will not score the jewel
        if (vuforiaHelper.isVisible())
        {
            boolean isLeftBlue = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("leftColor ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("RightColor ", vuforiaHelper.avgRightJewelColor);
            knockJewel(isLeftBlue, isBlueSide);
        }
        else
            telemetry.addData("vuMark: ", "not visible");

        telemetry.update();
        //--------------------------------------------------------------


        // Drive to safe zone-------------------------------------------
        if (isBlueSide)
        {
            if (isLeftBalancingStone)
            {
                driveToPosition(0, -610, 0.7);
                driveToPosition(-305, 0, 0.5);
            }
            else
                driveToPosition(0, -914, 0.7);
        }
        else
        {
            if (isLeftBalancingStone)
                driveToPosition(0, 914, 0.7);
            else
            {
                driveToPosition(0, 610, 0.7);
                driveToPosition(-305, 0, 0.5);
            }
        }
        //--------------------------------------------------------------

        // Score glyph in key column------------------------------------
        if (isBlueSide)
        {
            if (isLeftBalancingStone)
            {

            }
            else
            {

            }
        }
        else
        {
            if (isLeftBalancingStone)
            {

            }
            else
            {

            }
        }
        //---------------------------------------------------------------
    }
}
