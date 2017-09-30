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
        initializeHardware();

        waitForStart();

        //accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();

        while (opModeIsActive())
        {
            //finds the time elapsed each loop
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            //arcade drive; left stick rotates robot, right stick translates robot
            driveMecanumWithJoysticks();

            //updates that need to happen each loop
            telemetry.addData("eTime:", eTime);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }
}
