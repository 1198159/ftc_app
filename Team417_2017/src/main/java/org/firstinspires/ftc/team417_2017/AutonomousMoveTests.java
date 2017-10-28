package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Move Tests", group = "Swerve")
// @Disabled

public class AutonomousMoveTests extends MasterAutonomous
{

    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        super.initializeHardware();

        telemetry.addData("Path", "Select Team and Pos...");

        // Wait until we're told to go
        while (!isStarted())
                {
            // allow driver to choose a team
            if (gamepad1.b) isRedTeam = true;
            if (gamepad1.x) isRedTeam = false;

            // select position one or two, one is closer to the origin
            if (gamepad1.y) isStartingPosOne = true;
            if (gamepad1.a) isStartingPosOne = false;

            if (isRedTeam)
            {
                if (isStartingPosOne) telemetry.addData("Team: ", "Red 1");
                else telemetry.addData("Team: ", "Red 2");
            }
            else
            {
                if (isStartingPosOne) telemetry.addData("Team: ", "Blue 1");
                else telemetry.addData("Team: ", "Blue 2");
            }
            telemetry.update();
            idle();
        }
        telemetry.update();

        if (isRedTeam) // if team RED
        {
            if (isStartingPosOne)
            {

            }
            else
            {

            }
            telemetry.update();
        }
        else // if team BLUE
        {
            if (isStartingPosOne)
            {

            }
            else
            {

            }
        }

// Wait for the game to start (driver presses PLAY)
        waitForStart();
        autoRuntime.reset(); // set the 30 second timer

// START OF AUTONOMOUS

        Kmove = 1.0/1200.0;
        MINSPEED = 0.3;
        TOL_ANGLE = 3.0;
        TOL = 60;
        Kpivot = 1/150.0;
        PIVOT_MINSPEED = 0.2;

        double refAngle = imu.getAngularOrientation().firstAngle;
        //pivot(90, 0.1);
        //sleep(1000);
        pivotWithReference(-90, refAngle, 0.5);


        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
