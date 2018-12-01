package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 *  This program is an all-in-one autonomous.  It uses gamepad input prior to initialization to
 *  offer different routine options.
 */

@Autonomous(name = "AutoCompetition", group = "Autonomous")
//@Disabled

public class AutoCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        runSetup();

        waitForStart();
        // Wait to start the match for 0-10 seconds, depending on setup input.
        pauseWhileUpdating(matchDelay);

        // Crater-------------------------------------------------------------------------------
        if (isCraterStart)
        {
            //dropRobotAndUnlatch();

            // Detect minerals
            identifyGold();

            // Drive forward and knock off correct mineral
            knockGold(goldLocation);

            // Drive backward a small amount
            driveToPosition(0,-Constants.MINERAL_BACKWARD,0.5);

            // Turn 90 deg ccw
            turnTo(90,1.0);

            driveToDepotDropMarker();

            // Park in either our alliance or opponents' crater, depending on setup input.
            if(isAllianceCraterFinal)
            {
                knockPartnerMineral();
                turnTo(135,1.0);

                // Align with wall and drive backward into crater.
                moveRobot(0.0,0.7,0.7);
                driveToPosition(0, -1750, 1.0);
            }
            else
            {
                knockPartnerMineral();
                turnTo(45, 1.0);

                // Align with wall and drive backward into crater.
                moveRobot(180.0,0.7,0.7);
                driveToPosition(0, -1900, 1.0);
            }
        }
        // Depot--------------------------------------------------------------------------------
        else
        {
            dropRobotAndUnlatch();

            // Detect minerals
            identifyGold();

            // Drive forward and knock off correct mineral
            knockGold(goldLocation);

            // Turn toward depot and drive into it; change turn angle based on the location of the gold mineral.
            turnTo(turnShift, 1.0);
            driveToPosition(0,980 ,0.7);

            // Park in either our alliance's or opponents' crater, depending on setup input.
            if(isAllianceCraterFinal)
            {
                turnTo(45, 1.0);
                dropOffMarker();

                // Drive backward, align with wall, and drive rest of way into crater.
                driveToPosition(0, -1000, 1.0);
                moveRobot(0.0,0.6,0.4);
                driveToPosition(0, -750, 1.0);
            }
            else
            {
                turnTo(-45, 1.0);
                dropOffMarker();

                // Drive backward, align with wall, and drive rest of way into crater.
                driveToPosition(0, -1000, 1.0);
                moveRobot(180.0,0.6,1.0);
                driveToPosition(0, -900, 1.0);
            }
        }
        // -------------------------------------------------------------------------------------

        // Stop the vision system.
        OpenCVVision.disable();
    }

    // Functions that encapsulate lengthy, frequently used autonomous code.-------------------------
    private void dropRobotAndUnlatch() throws InterruptedException
    {
        // Drop robot to ground.
        motorHanger.setTargetPosition(Constants.HANG_UNLATCH_POSITION);
        motorHanger.setPower(1.0);
        pauseWhileUpdating(0.3);
        servoHanger.setPosition(Constants.SERVO_HANG_RETRACTED);
        motorHanger.setPower(0);
        pauseWhileUpdating(0.75);

        // Unlatch from hook while on ground, drive sideways, retract hanger, and return to original
        // position.
        motorHanger.setTargetPosition(Constants.HANG_GROUND_UNLATCH);
        motorHanger.setPower(1.0);
        driveToPosition(80,0,0.7);
        motorHanger.setTargetPosition(0);
        motorHanger.setPower(1.0);
        driveToPosition(-80,0,0.7);
    }

    // Allows us to score alliance partner's mineral.
    private void knockPartnerMineral() throws InterruptedException
    {
        // Only do this if we have pressed the proper button in autonomous.
        if(knockPartnerMineral)
        {
            if (goldLocation == sampleFieldLocations.right)
            {
                turnTo(135, 1.0);
                // Drive forward to hit mineral and return to original position.
                driveToPosition(0, -500, 1.0);
                driveToPosition(0, 500, 1.0);
            }
            else if (goldLocation == sampleFieldLocations.left)
            {
                turnTo(45, 1.0);
                // Drive forward to hit mineral and return to original position.
                driveToPosition(0, -500, 1.0);
                driveToPosition(0, 500, 1.0);
            }
            else if (goldLocation == sampleFieldLocations.center)
            {
                // Drive forward to hit mineral and return to original position.
                driveToPosition(0, -300, 1.0);
                driveToPosition(0, 300, 1.0);
            }



        }
    }

    // Code for spitting team marker out of collector.
    private void dropOffMarker() throws InterruptedException
    {
        servoMarker.setPosition(Constants.SERVO_MARKER_DEPLOYED);
        pauseWhileUpdating(0.5);
        servoMarker.setPosition(Constants.SERVO_MARKER_RETRACTED);
    }

    // Gives us the option to knock off our alliance partner's mineral if it is in the
    // right position, which is near the path of our robot anyway.
    private void driveToDepotDropMarker() throws InterruptedException
    {
        // Drive forward; we change this value based on where the gold mineral was.
        driveToPosition(0, 1250 + mineralShift, 1.0);
        turnTo(135,1.0);
        // Align robot with wall.
        moveRobot(0.0,0.7,0.5);
        // Drive forward.
        driveToPosition(0,1050,1.0);

        // Move robot away from wall.
        driveToPosition(-250,0,1.0);
        // Turn robot toward corner and drop marker.
        turnTo(90,1.0);
        dropOffMarker();
    }
    //----------------------------------------------------------------------------------------------
}
