package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Blue Left", group = "Autonomous")

public class AutoBlueLeft extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        boolean isBlueSide = true;
        boolean isLeftBalancingStone = true;
        setRobotStartingOrientation(180);

        initializeAuto();

        waitForStart();


        vuforiaHelper.getVumark();
        // Get jewel info-----------------------------------------------------
         // If the vuMark is not visible, vuforia will tell us
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


        // Drive to safe zone--------------------------------
        moveRobot(-90,0.5, 1.4);
        //---------------------------------------------------


        // Align with and face key column--------------------
        moveRobot(180,0.5, vuforiaHelper.keyColumnDriveTime(isBlueSide, isLeftBalancingStone));
        //driveToPosition(0, -500, 0.5);
        turnTo(0);
        //


        // Deploy glyph mechanism----------------------------
        motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(4.0);
        //---------------------------------------------------


        // Score glyph---------------------------------------
        motorCollectorRight.setPower(0.6);
        pauseWhileUpdating(1.0);
        motorCollectorRight.setPower(0);
        //---------------------------------------------------


        // Push glyph in-------------------------------------
        moveRobot(90, 0.2, 0.6);
        //---------------------------------------------------


        // Move robot away from cryptobox----------------
        moveRobot(-90, 0.3, 0.4);
        //-----------------------------------------------


        // Retract glyph mechanism---------------------------
        motorGlyphter.setTargetPosition(0);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(4.0);
        //---------------------------------------------------

    }
}
