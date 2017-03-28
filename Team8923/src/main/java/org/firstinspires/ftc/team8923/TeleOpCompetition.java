package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        // Initialize loop timers
        for(int i = 0; i < 9; i++)
            loopTimers[i] = new ElapsedTime();

        waitForStart();

        servoCollectorHolder.setPosition(ServoPositions.COLLECTOR_HOLDER_UP.pos);
        while(opModeIsActive())
        {
            loopTimers[LoopTimers.ALL_CALCULATIONS.ordinal()].reset();
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

            telemetry.addData("Calculation Time", formatNumber(loopTimers[LoopTimers.ALL_CALCULATIONS.ordinal()].milliseconds()));
            telemetry.addData("Loop Time", formatNumber(loopTimers[LoopTimers.LOOP_TIME.ordinal()].milliseconds()));
            loopTimers[LoopTimers.LOOP_TIME.ordinal()].reset();

            sendTelemetry();
            idle();
        }
    }
}
