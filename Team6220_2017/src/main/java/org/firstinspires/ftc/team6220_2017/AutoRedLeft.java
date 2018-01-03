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


        // Align with and face key column--------------------
        driveToPosition(0, vuforiaHelper.keyColumnDistance(isBlueSide, isLeftBalancingStone), 0.7);
        turnTo(-90);
        driveToPosition(0, -130, 0.4);
        //---------------------------------------------------


        // Deploy glyph mechanism----------------------------
        glyphMechanism.driveGlyphterToPosition(Constants.HEIGHT_1, 1.0);
        //---------------------------------------------------


        // Score glyph---------------------------------------
        motorCollectorLeft.setPower(-0.7);
        motorCollectorRight.setPower(0.4);
        pauseWhileUpdating(1.0);
        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);
        //---------------------------------------------------


        // Push glyph in-------------------------------------
        moveRobot(90, 0.2, 0.8);
        //---------------------------------------------------


        // Move robot away from cryptobox----------------
        moveRobot(-90, 0.3, 0.8);
        //-----------------------------------------------


        // Retract glyph mechanism---------------------------
        glyphMechanism.driveGlyphterToPosition(0, 1.0);
        //---------------------------------------------------


        // Move robot toward cryptobox----------------
        moveRobot(90, 0.3, 0.9);
        //-----------------------------------------------

        turnTo(90);

    }
}
