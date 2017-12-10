package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Red Left", group = "Autonomous")

public class AutoRedLeft extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        boolean isBlueSide = false;
        boolean isLeftBalancingStone = true;
        setRobotStartingOrientation(0);

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
        //driveToPosition(0, -500, 0.5);


        // Align with and face key column--------------------
        moveRobot(90, 0.5, vuforiaHelper.keyColumnDriveTime(isBlueSide, isLeftBalancingStone));
        //driveToPosition(0, -500, 0.5);
        turnTo(-90);
        //---------------------------------------------------


        // Deploy glyph mechanism----------------------------
        motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(3.0);
        //---------------------------------------------------


        // Move robot closer to cryptobox----------------
        moveRobot(0, 0.3, 1.4);
        //---------------------------------------------------


        // Score glyph---------------------------------------
        motorCollectorRight.setPower(0.6);
        pauseWhileUpdating(1.0);
        motorCollectorRight.setPower(0);
        //---------------------------------------------------
    }
}
