package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Auto Blue Right", group = "Autonomous")

public class AutoBlueRight extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        boolean isBlueSide = true;
        //setRobotStartingOrientation(180);

        initializeAuto();

        waitForStart();


        vuforiaHelper.getVumark();
        // Get jewel info-----------------------------------------------------
         // if the vuMark is not visible, vuforia will tell us
        if (vuforiaHelper.isVisible())
        {
            blueJewel = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("Left Hue: ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("Right Hue: ", vuforiaHelper.avgRightJewelColor);
        }
        else
        {
            telemetry.addData("vuMark: ", "not visible");
        }
        telemetry.update();
        //---------------------------------------------------------------------


        knockJewel(blueJewel, isBlueSide);
        moveRobot(-90,0.5,vuforiaHelper.positionOfGoal());
        turnTo(-90);
        //driveToPosition(0, -500, 0.5);
        //turn 90 degrees
        //may need to move backwards
        //deploy Glyph mech
        //score glyph
    }
}
