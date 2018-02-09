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
        // Move jewel servos so jewel jostler is out of the way of the glyph mechanism
        verticalJewelServoToggler.retract();
        lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_NEUTRAL);

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
        driveToPosition(0, -400, 0.4);
        //---------------------------------------------------


        // Deploy glyph mechanism----------------------------
        motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(2.6);
        //---------------------------------------------------

        // Score glyph---------------------------------------
        motorCollectorLeft.setPower(-0.6);
        motorCollectorRight.setPower(0.6);
        //---------------------------------------------------

        //  todo change all moveRobots to driveToPosition
        // Push glyph in-------------------------------------
        //moveRobot(90, 0.2, 0.5);
        //---------------------------------------------------

        // Move robot away from cryptobox and turn away----------------
        //moveRobot(-90, 0.3, 0.8);
        driveToPosition(0, -200, 0.4);

        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);

        motorGlyphter.setTargetPosition(Constants.HEIGHT_4);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(2.0);

        turnTo(90);
        driveToPosition(0, -360, 0.5);

        //-----------------------------------------------

        // Deploy glyph mechanism----------------------------
        motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(1.7);
        //---------------------------------------------------


        // Variable to make sure that we will go back to the cryptobox if we don't find any glyphs
        int collectionCount = 0;

        // Collect glyphs---------------------------------------
        motorCollectorLeft.setPower(0.6);
        motorCollectorRight.setPower(-0.6);
        //------------------------------------------------------

        // Drive until there is a glyph or until the robot goes too far forward--
        while(!isGlyph() && (collectionCount < 1))
        {
            driveToPosition(0, 620, 0.4);
            collectionCount++;
        }
        //-----------------------------------------------------------------------

        // Stop collector after robot has backed away from cryptobox to reduce risk of descoring glyphs
        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);

        // Back up and turn towards the cryptobox----------
        //moveRobot(-90, 0.4, 0.1);

        motorGlyphter.setTargetPosition(Constants.HEIGHT_4);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(1.4);

        turnTo(-90);
        //-------------------------------------------------

        //Move robot towards cryptobox and deploy glyph mechanism to 2nd height----
        driveToPosition(0, 430, 0.7);
        motorGlyphter.setTargetPosition(Constants.HEIGHT_3);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(1.0);    // todo Cut time?
        //--------------------------------------------------------------------------

        // Score glyphs--------------------------------------
        motorCollectorLeft.setPower(-0.6);
        motorCollectorRight.setPower(0.6);
        pauseWhileUpdating(0.5);
        //---------------------------------------------------

        // Move robot away from cryptobox----------------
        driveToPosition(0, -200, 0.4);
        //-----------------------------------------------

        // Stop collector after robot has backed away from cryptobox to reduce risk of descoring glyphs
        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);

        // Get ready for teleOp
        //turnTo(-90);

        // Retract glyph mechanism---------------------------
        //motorGlyphter.setTargetPosition(0);
        //motorGlyphter.setPower(1.0);
        //pauseWhileUpdating(4.0);
        //---------------------------------------------------
    }
}
