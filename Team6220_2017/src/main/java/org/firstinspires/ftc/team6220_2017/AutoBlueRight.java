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
        // Variable to make sure that we will go back to the cryptobox if we don't find any glyphs
        int collectionCount = 0;
        setRobotStartingOrientation(180);

        initializeAuto();

        waitForStart();
        // Move jewel servos so jewel jostler is out of the way of the glyph mechanism
        verticalJewelServoToggler.retract();
        lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_NEUTRAL);



        vuforiaHelper.getVumark();
        // Get jewel info---------------------------------------------------------------
        // if the vuMark is not visible, vuforia will tell us
        if (vuforiaHelper.isVisible()) {
            blueJewel = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("Left Hue: ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("Right Hue: ", vuforiaHelper.avgRightJewelColor);
        } else {
            telemetry.addData("vuMark: ", "not visible");
        }
        telemetry.update();
        //------------------------------------------------------------------------------



        //knockJewel(blueJewel, isBlueSide);



        // Drive to key column, turn around, and back up--------------------------------
        driveToPosition(0, -vuforiaHelper.keyColumnDistance(isBlueSide, isLeftBalancingStone), 0.65);  // todo Faster?

        turnTo(90);

        driveToPosition(0, -350, 0.6);
        //-------------------------------------------------------------------------------



        // Deploy glyph mechanism--------------------------------------------------------
        motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(3.3);
        //-------------------------------------------------------------------------------



        // Score glyph-------------------------------------------------------------------
        motorCollectorLeft.setPower(-0.5);
        motorCollectorRight.setPower(0.5);
        pauseWhileUpdating(0.3);
        //-------------------------------------------------------------------------------



        // Back away from cryptobox and stop collector-----------------------------------
        driveToPosition(0, -200, 0.5);

        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);
        //-------------------------------------------------------------------------------



        // Raise glyphter, turn toward glyph pile, and back up---------------------------
        motorGlyphter.setTargetPosition(Constants.HEIGHT_4);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(1.5);

        turnTo(-90);

        //moveRobot(-90, 0.8, 0.8);
        driveToPosition(0, -470, 0.7);
        //-------------------------------------------------------------------------------



        // Lower glyph mechanism for collection------------------------------------------
        motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(1.8);
        //-------------------------------------------------------------------------------



        // Collect glyphs----------------------------------------------------------------
        motorCollectorLeft.setPower(0.5);
        motorCollectorRight.setPower(-0.5);
        //-------------------------------------------------------------------------------



        // Drive until there is a glyph or until the robot goes too far forward----------
        while(!isGlyph() && (collectionCount < 1))
        {
            driveToPosition(0, 615, 0.7);
            collectionCount++;
        }
        //-------------------------------------------------------------------------------



        // Wait a short time for glyphs in tip of collector, then stop collecting--------
        //pauseWhileUpdating(0.2);
        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);
        //-------------------------------------------------------------------------------



        // Raise glyphter and turn slowly toward cryptobox to prevent flying glyphs------
        motorGlyphter.setTargetPosition(Constants.HEIGHT_4);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(1.4);    // todo Faster?

        adjustableTurnTo(90, 0.6);
        //-------------------------------------------------------------------------------



        // Move robot toward cryptobox and deploy glyph mechanism------------------------
        driveToPosition(0, 330, 0.7);

        motorGlyphter.setTargetPosition(Constants.HEIGHT_3);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(0.7);    // todo Faster?
        //-------------------------------------------------------------------------------



        // Score glyphs------------------------------------------------------------------
        motorCollectorLeft.setPower(-0.5);
        motorCollectorRight.setPower(0.5);
        pauseWhileUpdating(0.5);
        //-------------------------------------------------------------------------------



        // Back away quickly from cryptobox and stop collector---------------------------
        driveToPosition(0, -200, 0.5);

        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);
        //-------------------------------------------------------------------------------



        // Get ready for teleOp
        //turnTo(-90);
    }
}
