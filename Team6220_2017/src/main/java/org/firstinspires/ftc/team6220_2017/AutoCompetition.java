package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 *  This is our main autonomous program.  It uses gamepad input before initialization to offer
 *  four different routine options
 */

// todo Test setupRoutine
// todo Reimplement encoder navigation when it is ready
@Autonomous(name = "AutoCompetition", group = "Autonomous")

public class AutoCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        runSetup();

        waitForStart();
        // Move jewel servo so it is out of the way of the glyph mechanism
        verticalJewelServoToggler.retract();

        vuforiaHelper.getVumark();
        // Score jewel-------------------------------------------------
         // If the vuMark is not visible, vuforia will tell us and the robot will not score the jewel
        if (vuforiaHelper.isVisible())
        {
            blueJewel = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("Blue Jewel: ", blueJewel);

            knockJewel(blueJewel, isBlueSide);
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
                moveRobot(-90,0.5, vuforiaHelper.keyColumnDriveTime(isBlueSide, isLeftBalancingStone));
                //driveToPosition(0, -610, 0.7);
                //driveToPosition(-305, 0, 0.5);
            }
            else
            {
                moveRobot(-90, 0.5, 1.4);
                //driveToPosition(0, -914, 0.7);
            }
        }
        else
        {
            if (isLeftBalancingStone)
            {
                moveRobot(90,0.5, vuforiaHelper.keyColumnDriveTime(isBlueSide, isLeftBalancingStone));
                moveRobot(0, 0.3, 1.4);
                //driveToPosition(0, 914, 0.7);
            }
            else
            {
                moveRobot(90,0.5,1.4);
                //driveToPosition(0, 610, 0.7);
                //driveToPosition(-305, 0, 0.5);
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
