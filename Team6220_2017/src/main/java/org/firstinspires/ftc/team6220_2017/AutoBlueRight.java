package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "Auto Blue Right", group = "Autonomous")

public class AutoBlueRight extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        boolean isBlueSide = true;
        boolean isLeftBalancingStone = false;
        setRobotStartingOrientation(180);

        initializeAuto();

        waitForStart();

        vuforiaHelper.getVumark();
        // Get jewel info-----------------------------------------------------
        // if the vuMark is not visible, vuforia will tell us
        if (vuforiaHelper.isVisible()) {
            blueJewel = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("Left Hue: ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("Right Hue: ", vuforiaHelper.avgRightJewelColor);
        } else {
            telemetry.addData("vuMark: ", "not visible");
        }
        telemetry.update();
        //---------------------------------------------------------------------


        knockJewel(blueJewel, isBlueSide);


        // Align with and face key column--------------------
        driveToPosition(0, -vuforiaHelper.keyColumnDistance(isBlueSide, isLeftBalancingStone), 0.7);
        turnTo(90);
        driveToPosition(0, -130, 0.4);
        //---------------------------------------------------


        // Deploy glyph mechanism----------------------------
        motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(4.0);
        //---------------------------------------------------


        // Score glyph---------------------------------------
        motorCollectorLeft.setPower(-0.6);
        motorCollectorRight.setPower(0.6);
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


        turnTo(-90);


        int collectionCount = 0;
        // Collect glyphs---------------------------------------
        motorCollectorLeft.setPower(0.6);
        motorCollectorRight.setPower(-0.6);
        //------------------------------------------------------
        while(!isGlyph() && (collectionCount < 4))
        {
            driveToPosition(0, 500, 0.4);
            collectionCount++;
        }
        // Wait a short time for glyphs in tip of collector, then stop collecting---
        pauseWhileUpdating(0.2);
        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);
        //--------------------------------------------------------------------------

        // Back up
        moveRobot(-90, 0.3, 0.8);

        turnTo(90);

        //Move robot towards cryptobox and deploy glyph mechanism to 2nd height----
        driveToPosition(0, (500 * collectionCount), 1.0);
        motorGlyphter.setTargetPosition(Constants.HEIGHT_2);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(4.0);
        //--------------------------------------------------------------------------

        // Score glyphs--------------------------------------
        motorCollectorLeft.setPower(-0.6);
        motorCollectorRight.setPower(0.6);
        pauseWhileUpdating(1.0);
        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);
        //---------------------------------------------------

        // Move robot away from cryptobox----------------
        moveRobot(-90, 0.3, 0.8);
        //-----------------------------------------------

        // Get ready for teleOp
        turnTo(-90);

        // Retract glyph mechanism---------------------------
        //motorGlyphter.setTargetPosition(0);
        //motorGlyphter.setPower(1.0);
        //pauseWhileUpdating(4.0);
        //---------------------------------------------------
    }
}
