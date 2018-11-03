package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Const;

import java.util.Locale;

/**
 *  Note:  This program is not currently in use.  It may be useful as a template for the future, however.
 *  This program uses gamepad input prior to initialization to offer different routine options.
 */

// todo Update code
// todo Reimplement encoder navigation when it is ready
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

        // Blue side------------------------------------------------------------------------
        if (isBlueSide)
        {
            // Blue + Crater-----------------------------------------------------------------------
            if (isCraterStart)
            {
                // Drop robot to ground
                motorHanger.setTargetPosition(Constants.HANG_UNLATCH_POSITION);
                motorHanger.setPower(1.0);
                pauseWhileUpdating(0.3);
                servoHanger.setPosition(Constants.SERVO_HANG_RETRACTED);
                motorHanger.setPower(0);
                pauseWhileUpdating(1.5);

                // Turn and detect minerals
                identifyGold();

                // Unlatch from hook while on ground
                motorHanger.setTargetPosition(Constants.HANG_GROUND_UNLATCH);
                motorHanger.setPower(1.0);
                driveToPosition(60,0,0.7);
                motorHanger.setTargetPosition(0);
                motorHanger.setPower(1.0);
                driveToPosition(-60,0,0.7);

                // Drive forward and knock off correct mineral
                knockGold(goldLocation);

                // Drive backward a small amount
                driveToPosition(0,-Constants.MINERAL_BACKWARD,0.5);

                // Turn 90 deg ccw
                turnTo(90,1.0);

                // Drive forward; we change this value based on where the gold mineral was
                driveToPosition(0,1250 + mineralShift,0.7);

                // Turn 45 deg ccw
                turnTo(135,1.0);

                // Align robot with wall now if we do not want to knock our partner's mineral.
                if (!knockPartnerMineral)
                    moveRobot(0.0,0.6,0.8);

                // Drive forward
                driveToPosition(0,1100,0.7);

                // moveRobot forward quickly, moveRobot backward quickly to drop team marker
                moveRobot(90,1.0,0.2);
                moveRobot(-90,1.0,0.2);

                // Align robot with wall now if we did knock our partner's mineral.
                if (knockPartnerMineral)
                    moveRobot(0.0,0.6,0.8);

                // Drive backward into crater
                driveToPosition(0,-1850,1.0);
            }
            // Blue + Depot-----------------------------------------------------------------------
            else
            {
                // Drop robot to ground
                motorHanger.setTargetPosition(Constants.HANG_UNLATCH_POSITION);
                motorHanger.setPower(1.0);
                pauseWhileUpdating(0.3);
                servoHanger.setPosition(Constants.SERVO_HANG_RETRACTED);
                motorHanger.setPower(0);
                pauseWhileUpdating(1.5);

                // Turn and detect minerals
                identifyGold();

                // Unlatch from hook while on ground
                motorHanger.setTargetPosition(Constants.HANG_GROUND_UNLATCH);
                motorHanger.setPower(1.0);
                driveToPosition(60,0,0.7);
                motorHanger.setTargetPosition(0);
                motorHanger.setPower(1.0);
                driveToPosition(-60,0,0.7);

                // Drive forward and knock off correct mineral
                knockGold(goldLocation);

                // Turn robot toward depot based on gold mineral position
                if (goldLocation == sampleFieldLocations.left)
                    turnTo(-30,0.7);
                else if (goldLocation == sampleFieldLocations.right)
                    turnTo(30,0.7);

                // Drive forward into depot
                driveToPosition(0,850,0.7);

                // moveRobot forward quickly, moveRobot backward quickly to drop team marker
                moveRobot(90,1.0,0.2);
                moveRobot(-90,1.0,0.2);

                // Orient robot with rear toward crater
                turnTo(-45,1.0);

                // Drive backward out of depot
                driveToPosition(0,-1000,1.0);

                // Align robot with wall
                moveRobot(180.0,0.6,0.8);

                // Drive backward into crater
                driveToPosition(0,-850,1.0);
            }
        }
        // Red side-------------------------------------------------------------------------
        else
        {
            // Red + Crater-----------------------------------------------------------------------
            if (isCraterStart)
            {
                // Drop robot to ground
                motorHanger.setTargetPosition(Constants.HANG_UNLATCH_POSITION);
                motorHanger.setPower(1.0);
                pauseWhileUpdating(0.3);
                servoHanger.setPosition(Constants.SERVO_HANG_RETRACTED);
                motorHanger.setPower(0);
                pauseWhileUpdating(1.5);

                // Turn and detect minerals
                identifyGold();

                // Unlatch from hook while on ground
                motorHanger.setTargetPosition(Constants.HANG_GROUND_UNLATCH);
                motorHanger.setPower(1.0);
                driveToPosition(60,0,0.7);
                motorHanger.setTargetPosition(0);
                motorHanger.setPower(1.0);
                driveToPosition(-60,0,0.7);

                // Drive forward and knock off correct mineral
                knockGold(goldLocation);

                // Drive backward a small amount
                driveToPosition(0,-Constants.MINERAL_BACKWARD,0.5);

                // Turn 90 deg ccw
                turnTo(90,1.0);

                // Drive forward; we change this value based on where the gold mineral was
                driveToPosition(0,1250 + mineralShift,0.7);

                // Turn 45 deg ccw
                turnTo(135,1.0);

                // Align robot with wall now if we do not want to knock our partner's mineral.
                if (!knockPartnerMineral)
                    moveRobot(0.0,0.6,0.8);

                // Drive forward
                driveToPosition(0,1100,0.7);

                // moveRobot forward quickly, moveRobot backward quickly to drop team marker
                moveRobot(90,1.0,0.2);
                moveRobot(-90,1.0,0.2);

                // Align robot with wall now if we did knock our partner's mineral.
                if (knockPartnerMineral)
                    moveRobot(0.0,0.6,0.8);

                // Drive backward into crater
                driveToPosition(0,-1850,1.0);
            }
            // Red + Depot-----------------------------------------------------------------------
            else
            {
                // Drop robot to ground
                motorHanger.setTargetPosition(Constants.HANG_UNLATCH_POSITION);
                motorHanger.setPower(1.0);
                pauseWhileUpdating(0.3);
                servoHanger.setPosition(Constants.SERVO_HANG_RETRACTED);
                motorHanger.setPower(0);
                pauseWhileUpdating(1.5);

                // Turn and detect minerals
                identifyGold();

                // Unlatch from hook while on ground
                motorHanger.setTargetPosition(Constants.HANG_GROUND_UNLATCH);
                motorHanger.setPower(1.0);
                driveToPosition(60,0,0.7);
                motorHanger.setTargetPosition(0);
                motorHanger.setPower(1.0);
                driveToPosition(-60,0,0.7);

                // Drive forward and knock off correct mineral
                knockGold(goldLocation);

                // Turn robot toward depot based on gold mineral position
                if (goldLocation == sampleFieldLocations.left)
                    turnTo(-30,0.7);
                else if (goldLocation == sampleFieldLocations.right)
                    turnTo(30,0.7);

                // Drive forward into depot
                driveToPosition(0,850,0.7);

                // moveRobot forward quickly, moveRobot backward quickly to drop team marker
                moveRobot(90,1.0,0.2);
                moveRobot(-90,1.0,0.2);

                // Orient robot with rear toward crater
                turnTo(-45,1.0);

                // Drive backward out of depot
                driveToPosition(0,-1000,1.0);

                // Align robot with wall
                moveRobot(180.0,0.6,0.8);

                // Drive backward into crater
                driveToPosition(0,-850,1.0);
            }
        }

        // Stop the vision system.
        OpenCVVision.disable();
    }
}
