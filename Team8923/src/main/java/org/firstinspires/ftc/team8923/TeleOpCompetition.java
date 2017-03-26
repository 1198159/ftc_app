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

        servoCollectorHolder.setPosition(ServoPositions.COLLECTOR_HOLDER_UP.pos);
        loopTimer.reset();

        while(opModeIsActive())
        {
            lastLoopTimes[0] = loopTimer.milliseconds();
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
            telemetry.addData("","");
            telemetry.addData("Loop Cycle (ms) = ", ((int)((loopTimer.milliseconds() - lastLoopTimes[0]) / 10.0)) * 10.0);
            telemetry.addData("Drive Loop Time", ((int)((currentLoopTimes[1] - lastLoopTimes[1]) / LOOP_TIME_CONSTANT)) * LOOP_TIME_CONSTANT);
            telemetry.addData("Beacon Pusher Loop Time", ((int)((currentLoopTimes[2] - lastLoopTimes[2]) / LOOP_TIME_CONSTANT)) * LOOP_TIME_CONSTANT);
            telemetry.addData("Lift Control Loop Time", ((int)((currentLoopTimes[3] - lastLoopTimes[3]) / LOOP_TIME_CONSTANT)) * LOOP_TIME_CONSTANT);
            telemetry.addData("Collector Loop Time", ((int)((currentLoopTimes[4] - lastLoopTimes[4]) / LOOP_TIME_CONSTANT)) * LOOP_TIME_CONSTANT);
            telemetry.addData("Hopper Control Loop Time", ((int)((currentLoopTimes[5] - lastLoopTimes[5]) / LOOP_TIME_CONSTANT)) * LOOP_TIME_CONSTANT);
            telemetry.addData("Catapult Control Loop Time", ((int)((currentLoopTimes[6] - lastLoopTimes[6]) / LOOP_TIME_CONSTANT)) * LOOP_TIME_CONSTANT);
            telemetry.addData("Auto Catapult Firing Loop Time", ((int)((currentLoopTimes[7] - lastLoopTimes[7]) / LOOP_TIME_CONSTANT)) * LOOP_TIME_CONSTANT);
            telemetry.addData("","");
        }
    }
}
