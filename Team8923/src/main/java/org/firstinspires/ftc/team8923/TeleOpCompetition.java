package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This class contains code for running the robot with all driver controls
 */
@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{
    private boolean capBallDriveMode = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();

        waitForStart();

        while(opModeIsActive())
        {
            /* TODO: Add back in when cap ball drive mode has been fixed
            // Select drive mode
            if(gamepad1.dpad_down)
                capBallDriveMode = true;
            else if(gamepad1.dpad_up)
                capBallDriveMode = false;

            // Run drive mode
            if(!capBallDriveMode)
                driveMecanumTeleOp();
            else if(capBallDriveMode)
                driveAroundCapBall();*/
            driveMecanumTeleOp();

            // Run all of the mechanisms
            controlBeaconPusher();
            runLift();
            runCollector();
            controlCatapult();
            controlHopper();

            sendTelemetry();
            idle();
        }
    }
}
