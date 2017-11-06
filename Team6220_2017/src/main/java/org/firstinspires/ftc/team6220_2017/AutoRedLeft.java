package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Mridula on 11/1/2017.
 */
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
        // must initialize to prevent errors; not necessarily true
        boolean isLeftBlue = true;


        vuforiaHelper.getVumark();


        //if the vuMark is not visible, vuforia will tell us
        if (vuforiaHelper.isVisible())
        {
            isLeftBlue = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("leftColor ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("RightColor ", vuforiaHelper.avgRightJewelColor);
        }
        else
        {
            telemetry.addData("vuMark: ", "not visible");
        }

        telemetry.update();


        // auto code-----------------------------
        knockJewel(isLeftBlue, isBlueSide);

        //moveRobot(0, 1, 1000);
        //---------------------------------------
    }
}
