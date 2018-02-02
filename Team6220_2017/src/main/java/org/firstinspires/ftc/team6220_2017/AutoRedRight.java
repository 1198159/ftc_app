package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Mridula on 10/29/2017.
 */
@Autonomous(name = "Auto Red Right", group = "Autonomous")

public class AutoRedRight extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        boolean isBlueSide = false;
        boolean isLeftBalancingStone = false;
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


        moveRobot(90, 0.3, 1.2);
        turnTo(0);
        moveRobot(180, 0.6, vuforiaHelper.keyColumnDistance(isBlueSide,isLeftBalancingStone));
        turnTo(0);
        //moveRobot(180, 0.6, 0.5);


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

        moveRobot(-90, 0.3, 0.5);
        // Retract glyph mechanism---------------------------
        //motorGlyphter.setTargetPosition(0);
        //motorGlyphter.setPower(1.0);
        //pauseWhileUpdating(4.0);
        //---------------------------------------------------

        // Move robot toward cryptobox----------------
        moveRobot(90, 0.3, 0.75);
        //-----------------------------------------------
    }
}

