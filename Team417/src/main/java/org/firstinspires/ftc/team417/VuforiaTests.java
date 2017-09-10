package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Vuforia Tests", group = "Swerve")
// @Disabled

public class VuforiaTests extends MasterAutonomous
{

    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        VuforiaNav.initVuforia();
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
                // OPTION RED ONE (TOOLS)
                startDelay = 2000;
                pivotAngle = 55; // pivot this amount before acquiring target
                targetAngle = 0; // Vuforia angle
                startDist = 2286;
                targetIndex = 1;
                targetPos[0] = 2743.2f;
                targetPos[1] = mmFTCFieldWidth;
            }
            else
            {
                // OPTION RED TWO (GEARS)
                startDelay = 0;
                pivotAngle = 55; // pivot this amount before acquiring target
                targetAngle = 0; // Vuforia angle
                startDist = 1397;
                targetIndex = 3;
                targetPos[0] = 1524;
                targetPos[1] = mmFTCFieldWidth;
            }
            telemetry.update();
        }
        else // if team BLUE
        {
            if (isStartingPosOne)
            {
                // OPTION BLUE ONE (LEGOS)
                startDelay = 2000;
                pivotAngle = -55;
                targetAngle = -90;
                startDist = 2286;
                targetIndex = 2;
                targetPos[0] = mmFTCFieldWidth;
                targetPos[1] = 2743.2f;
            }
            else
            {
                // OPTION BLUE TWO (WHEELS)
                startDelay = 0;
                pivotAngle = -55;
                targetAngle = -90;
                startDist = 1397;
                targetIndex = 0;
                targetPos[0] = mmFTCFieldWidth;
                targetPos[1] = 1524;
            }
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        autoRuntime.reset(); // set the 30 second timer
        VuforiaNav.startTracking();
        VuforiaNav.getLocation(targetIndex);


// START OF AUTONOMOUS

        TOL = 30;
        VUFORIA_TOL = 10;
        TOL_ANGLE = 3.0;
        VUFORIA_TOL_ANGLE = 3.0;
        Kmove = 1.0/1000.0;
        Kpivot = 1.0/200.0;
        MINSPEED = 0.6;

        // setting for align pivot Vuforia
        TOL_ANGLE = 3.0;
        VUFORIA_TOL_ANGLE = 3.0;
        MINSPEED = 0.55;

        telemetry.addData("Path", "align pivot vuf");
        telemetry.update();
        alignPivotVuforia(0.6, 10, 600, 4);
        pause(50);

        do
        {
            VuforiaNav.getLocation(targetIndex); // update target location and angle
        }
        while (VuforiaNav.lastLocation == null);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
