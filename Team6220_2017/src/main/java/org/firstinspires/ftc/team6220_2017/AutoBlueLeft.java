package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Const;

@Autonomous(name = "Auto Blue Left", group = "Autonomous")

public class AutoBlueLeft extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        waitForStart();

        //setRobotStartingOrientation(180);

        boolean isBlueSide = true;
        // Must initialize to prevent errors; not necessarily true
        boolean isLeftBlue = true;


        vuforiaHelper.getVumark();


        // If the vuMark is not visible, vuforia will tell us
        if (vuforiaHelper.isVisible())
        {
            isLeftBlue = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("leftColor ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("RightColor ", vuforiaHelper.avgRightJewelColor);

            knockJewel(isLeftBlue, isBlueSide);
        }
        else
        {
            telemetry.addData("vuMark: ", "not visible");
        }
        telemetry.update();
    }
}
