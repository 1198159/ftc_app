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
        // Variable to make sure that we will go back to the cryptobox if we don't find any glyphs
        int collectionCount = 0;
        setRobotStartingOrientation(0);

        initializeAuto();

        waitForStart();
        // Move jewel servos so jewel jostler is out of the way of the glyph mechanism
        verticalJewelServoToggler.retract();
        lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_NEUTRAL);



        vuforiaHelper.getVumark();
        // Get jewel info--------------------------------------------------------------
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
        //------------------------------------------------------------------------------



        knockJewel(blueJewel, isBlueSide);



        // Drive to key column, turn, and score glyph with glyph clip--------------------
        driveToPosition(0, vuforiaHelper.keyColumnDistance(isBlueSide, isLeftBalancingStone), 0.75);  // todo Faster?
        //turnTo(-90);
        turnTo(75);

        glyphClipServoToggler.toggle();
        driveToPosition(0, -115, 0.5);
        //-------------------------------------------------------------------------------
        turnTo(90);
        moveRobot(-90, 0.4, 0.3);
        turnTo(90);



        // Lower glyph mechanism for collection------------------------------------------
        motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(3.6);
        //-------------------------------------------------------------------------------



        // Collect glyphs----------------------------------------------------------------
        motorCollectorLeft.setPower(0.5);
        motorCollectorRight.setPower(0.5);
        //-------------------------------------------------------------------------------



        // Drive until there is a glyph or until the robot goes too far forward----------
        while(!isGlyph() && (collectionCount < 1))
        {
            driveToPosition(0, 600, 0.7);
            collectionCount++;
        }
        //-------------------------------------------------------------------------------



        //turnTo(82);
        turnTo(98);
        turnTo(90);



        // Wait a short time for glyphs in tip of collector, then stop collecting--------
        //pauseWhileUpdating(0.2);
        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);
        //-------------------------------------------------------------------------------



        // Back up, Raise glyphter, and turn slowly toward cryptobox to prevent flying glyphs---
        driveToPosition(0, -150, 0.6);
        motorGlyphter.setTargetPosition(Constants.HEIGHT_4);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(1.4);    // todo Faster?

        adjustableTurnTo(-90, 0.7);

        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);
        //-------------------------------------------------------------------------------



        // Move robot toward cryptobox and deploy glyph mechanism------------------------
        driveToPosition(0, 150, 0.6);

        turnTo(-104);

        motorGlyphter.setTargetPosition(Constants.HEIGHT_3);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(0.6);    // todo Faster?
        //-------------------------------------------------------------------------------



        // Score glyphs------------------------------------------------------------------
        motorCollectorLeft.setPower(-0.5);
        motorCollectorRight.setPower(-0.5);
        pauseWhileUpdating(0.5);
        //-------------------------------------------------------------------------------



        driveToPosition(0, -180, 0.6);

        motorCollectorLeft.setPower(0.0);
        motorCollectorRight.setPower(0.0);

        motorGlyphter.setTargetPosition(Constants.HEIGHT_4);
        pauseWhileUpdating(1.0);    // todo Adjust



        // Get ready for teleOp
        turnTo(90);
        driveToPosition(0, -250, 0.5);
        motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
        pauseWhileUpdating(2.4);    // todo Adjust
    }
}
