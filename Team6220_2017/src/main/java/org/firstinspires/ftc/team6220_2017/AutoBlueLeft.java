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


        // Move off balancing stone and turn around--------
        moveRobot(-90, 0.3, 1.55);
        pause(300);
        turnTo(0);
        //-------------------------------------------------


        // Back up to edge of balancing stone--------------
        moveRobot(-90, 0.3, 0.55);
        turnTo(0);
        //-------------------------------------------------


        // Line up with key column-------------------------
        moveRobot(0, 0.6, vuforiaHelper.keyColumnDriveTime(isBlueSide,isLeftBalancingStone));
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
        motorCollectorRight.setPower(0.4);
        pauseWhileUpdating(1.0);
        motorCollectorLeft.setPower(0);
        motorCollectorRight.setPower(0);
        //---------------------------------------------------


        // Back away from cryptobox--------------------------
        moveRobot(-90, 0.3, 0.6);
        turnTo(0);
        //---------------------------------------------------


        // Retract glyph mechanism---------------------------
        motorGlyphter.setTargetPosition(0);
        motorGlyphter.setPower(1.0);
        pauseWhileUpdating(4.0);
        //---------------------------------------------------

        // Move robot toward cryptobox----------------
        moveRobot(90, 0.3, 0.75);
        //-----------------------------------------------
    }
}
