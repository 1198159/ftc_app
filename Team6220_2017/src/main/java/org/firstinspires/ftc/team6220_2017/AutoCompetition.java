package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 *  Note:  This program is not currently in use.  It may be useful as a template for the future, however.
 *  This program uses gamepad input prior to initialization to offer four different routine options.
 */

// todo Update code
// todo Reimplement encoder navigation when it is ready
@Autonomous(name = "AutoCompetition", group = "Autonomous")
@Disabled

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
        // Score jewel------------------------------------------
        if (vuforiaHelper.isVisible())
        {
            blueJewel = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("Blue Jewel: ", blueJewel);
        }
        else
        {
            telemetry.addData("vuMark: ", "not visible");
        }
        telemetry.update();


// Score jewel based on alliance and jewel colors.  If the vuMark is not visible, knockJewel
// will do nothing rather than score the jewel blindly
        knockJewel(blueJewel, isBlueSide);
//--------------------------------------------------------------


// Drive to safe zone-------------------------------------------
        if (isBlueSide)
        {
            if (isLeftBalancingStone)
            {
                // Move off balancing stone and turn around--------
                moveRobot(-90, 0.3, 1.55);
                pauseWhileUpdating(0.3);
                turnTo(0);
                //-------------------------------------------------


                // Back up to edge of balancing stone--------------
                moveRobot(-90, 0.3, 0.55);
                turnTo(0);
                //-------------------------------------------------


                // Line up with key column-------------------------
                moveRobot(0, 0.6, vuforiaHelper.keyColumnDistance(isBlueSide,isLeftBalancingStone));
                turnTo(0);
                //moveRobot(180, 0.6, 0.5);
                //-------------------------------------------------
            }
            else
            {
                // Align with and face key column--------------------
                moveRobot(-90, 0.5, vuforiaHelper.keyColumnDistance(isBlueSide, isLeftBalancingStone));
                //driveToPosition(0, -500, 0.5);
                turnTo(90);
                moveRobot(-90, 0.3, 0.7);
                // Ensure the robot is at the correct angle to score
                turnTo(90);
                //---------------------------------------------------
            }
        }
        else
        {
            if (isLeftBalancingStone)
            {
                // Align with and face key column--------------------
                moveRobot(90, 0.5, vuforiaHelper.keyColumnDistance(isBlueSide, isLeftBalancingStone));
                //driveToPosition(0, -500, 0.5);
                turnTo(-90);
                moveRobot(-90, 0.3, 0.7);
                // Ensure the robot is at the correct angle to score
                turnTo(-90);
                //---------------------------------------------------
            }
            else
            {
                moveRobot(90, 0.3, 1.2);
                turnTo(0);
                moveRobot(180, 0.6, vuforiaHelper.keyColumnDistance(isBlueSide,isLeftBalancingStone));
                turnTo(0);
                //moveRobot(180, 0.6, 0.5);
            }
        }
//--------------------------------------------------------------


// Score glyph in key column------------------------------------
        if (isBlueSide)
        {
            if (isLeftBalancingStone)
            {
                // Deploy glyph mechanism----------------------------
                motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
                motorGlyphter.setPower(1.0);
                pauseWhileUpdating(4.0);
                //---------------------------------------------------


                // Score glyph---------------------------------------
                motorCollectorLeft.setPower(-0.7);
                motorCollectorRight.setPower(0.4);
                pauseWhileUpdating(1.0);
                motorCollectorLeft.setPower(0);
                motorCollectorRight.setPower(0);
                //---------------------------------------------------


                // Back away from cryptobox--------------------------
                moveRobot(-90, 0.3, 0.6);
                turnTo(0);
                //---------------------------------------------------
            }
            else
            {
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
            }
        }
        else
        {
            if (isLeftBalancingStone)
            {
                // Deploy glyph mechanism----------------------------
                motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
                motorGlyphter.setPower(1.0);
                pauseWhileUpdating(4.0);
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
            }
            else
            {
                // Deploy glyph mechanism----------------------------
                motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
                motorGlyphter.setPower(1.0);
                pauseWhileUpdating(4.0);
                //---------------------------------------------------


                // Score glyph---------------------------------------
                motorCollectorLeft.setPower(-0.7);
                motorCollectorRight.setPower(0.4);
                pauseWhileUpdating(1.0);
                motorCollectorLeft.setPower(0);
                motorCollectorRight.setPower(0);
                //---------------------------------------------------


                // Back away from cryptobox--------------------------
                moveRobot(-90, 0.3, 0.6);
                turnTo(0);
                //---------------------------------------------------
            }
        }
//---------------------------------------------------------------



    }
}
