package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * This class contains code for running the CapBot during Autonomous. It's been designed so the
 * drivers can pick the objectives for it to complete.
 */
@Autonomous(name = "First Auto", group = "Autonomous")
public class AutonomousBlue extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        // TODO: Add code to pick objectives

        initHardware();

        waitForStart();

        vuforiaLocator.startTracking();

        while(opModeIsActive())
        {
            updateRobotLocation();

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Angle", robotAngle);
            telemetry.update();
            idle();
        }

        // TODO: Add code to go to objectives
    }
}
