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
        // Move jewel servos so jewel jostler is out of the way of the glyph mechanism
        verticalJewelServoToggler.retract();
        lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_NEUTRAL);

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


        // Move off balancing stone and turn---------------
        driveToPosition(0, -450, 0.7);  // todo Adjust y distance
        turnTo(90);
        //moveRobot(-90, 0.3, 1.55);
        //pauseWhileUpdating(0.3);
        //turnTo(0);
        //-------------------------------------------------



        // Navigate to key column using Vuforia and turn--- // todo Adjust keyColumnDistance
        driveToPosition(0, -vuforiaHelper.keyColumnDistance(isBlueSide, isLeftBalancingStone), 0.5);  // todo Faster?

        turnTo(180);
        //-------------------------------------------------



        // Score glyph-------------------------------------
        glyphClipServoToggler.toggle();
        driveToPosition(0, -130, 0.5);  // todo Adjust y distance
        driveToPosition(0, 130, 0.5);
        //-------------------------------------------------

        /*
        // Back up to edge of balancing stone--------------
        moveRobot(-90, 0.3, 0.55);
        turnTo(0);
        //-------------------------------------------------


        // Line up with key column-------------------------
        moveRobot(0, 0.6, vuforiaHelper.keyColumnDistance(isBlueSide,isLeftBalancingStone));
        turnTo(0);
        //moveRobot(180, 0.6, 0.5);
        //-------------------------------------------------


        // Deploy glyph mechanism----------------------------
        motorGlyphter.setTargetPosition(Constants.HEIGHT_1);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(4.0);
        //---------------------------------------------------

        // Score glyph---------------------------------------
        motorCollectorLeft.setPower(-0.7);
        motorCollectorRight.setPower(-0.4);
        pauseWhileUpdating(1.0);
        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);
        //---------------------------------------------------


        // Back away from cryptobox--------------------------
        moveRobot(-90, 0.3, 0.6);
        turnTo(0);
        //---------------------------------------------------


        // Retract glyph mechanism---------------------------
        // motorGlyphter.setTargetPosition(0);
        //motorGlyphter.setPower(1.0);
        //pauseWhileUpdating(4.0);
        //---------------------------------------------------

        // Move robot toward cryptobox----------------
        moveRobot(90, 0.3, 0.75);
        //-----------------------------------------------
        */
    }
}
