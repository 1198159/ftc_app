package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 *  uses driver input to operate robot
 */

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        waitForStart();

        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();

        while (opModeIsActive())
        {
            // Finds the time elapsed each loop
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            // 1st driver input for moving robot; left stick rotates robot, right stick translates robot
            driveMecanumWithJoysticks();
            // 2nd driver input for relic arm; right stick moves arm
            armMechanism.driveArm();

            /*
             updates that need to happen each loop
             note:  eTime is not currently displayed (it interrupts other telemetry), but it may
             be useful later
            */
            //telemetry.addData("eTime:", eTime);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }
}
