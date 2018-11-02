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
                /*// Drop robot to ground
                motorHanger.setTargetPosition(Constants.HANG_UNLATCH_POSITION);
                motorHanger.setPower(1.0);
                pauseWhileUpdating(0.3);
                servoHanger.setPosition(Constants.SERVO_HANG_RETRACTED);
                motorHanger.setPower(0);
                pauseWhileUpdating(2.0);
                // Turn while detecting minerals
                //turnTo(0,0.7);
                identifyGold();

                // Unlatch from hook while on ground
                motorHanger.setTargetPosition(Constants.HANG_GROUND_UNLATCH);
                motorHanger.setPower(1.0);
                driveToPosition(60,0,0.7);
                motorHanger.setTargetPosition(0);
                motorHanger.setPower(1.0);
                driveToPosition(-60,0,0.7);*/

                // Drive forward and knock off correct mineral
                knockGold(goldLocation);

                // Drive backward a small amount
                driveToPosition(0,Constants.MINERAL_BACKWARD,0.8);

                // Turn 90 deg ccw
                turnTo(90,1.0);

                // Drive forward; we change this value based on where the gold mineral was
                driveToPosition(0,1400 + mineralShift,1.0);

                // Turn 45 deg ccw
                turnTo(135,1.0);

                // Drive forward
                driveToPosition(0,1140,1.0);

                // moveRobot forward quickly, moveRobot backward quickly
                moveRobot(90,1.0,0.2);
                moveRobot(-90,1.0,0.2);

                // Drive backward until robot hits crater
                driveToPosition(0,-1830,1.0);
            }
            // Blue + Depot-----------------------------------------------------------------------
            else
            {
                identifyGold();
                // Drive forward and knock off correct mineral
                knockGold(goldLocation);
                while (opModeIsActive())
                    idle();
            }
        }
        // Red side-------------------------------------------------------------------------
        else
        {
            // Red + Crater-----------------------------------------------------------------------
            if (isCraterStart)
            {

            }
            // Red + Depot-----------------------------------------------------------------------
            else
            {

            }

        }

        // Stop the vision system.
        OpenCVVision.disable();
    }
}
