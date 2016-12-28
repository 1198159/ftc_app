package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Tests 2", group = "Swerve")
// @Disabled

public class AutonomousTests2 extends MasterAutonomous
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
            if (gamepad1.b)
            {
                isRedTeam = true;
            }

            if (gamepad1.x)
            {
                isRedTeam = false;
            }

            // select position one or two, one is closer to the origin
            if (gamepad1.y)
            {
                isStartingPosOne = true;
            }
            if (gamepad1.a)
            {
                isStartingPosOne = false;
            }

            if (isRedTeam)
            {
                if (isStartingPosOne)
                {
                    telemetry.addData("Team: ", "Red 1");
                }
                else
                {
                    telemetry.addData("Team: ", "Red 2");
                }
            }
            else
            {
                if (isStartingPosOne)
                {
                    telemetry.addData("Team: ", "Blue 1");
                }
                else
                {
                    telemetry.addData("Team: ", "Blue 2");
                }
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
                //telemetry.addData("Team: ", "Red 1"); // display what team we're on after choosing with the buttons
            }
            else
            {
                // OPTION RED TWO (GEARS)
                startDelay = 0;
                pivotAngle = 55; // pivot this amount before acquiring target
                targetAngle = 0; // Vuforia angle
                startDist = 1270;
                targetIndex = 3;
                targetPos[0] = 1524;
                targetPos[1] = mmFTCFieldWidth;
                //telemetry.addData("Team: ", "Red 2"); // display what team we're on after choosing with the buttons
            }
            telemetry.update();
        }
        else // if team BLUE
        {
            if (isStartingPosOne)
            {
                // OPTION BLUE ONE (LEGOS)
                startDelay = 2000;
                pivotAngle = -55; // recalc pivot?? also for red one??
                targetAngle = -90;
                startDist = 2286;
                targetIndex = 2;
                targetPos[0] = mmFTCFieldWidth;
                targetPos[1] = 2743.2f;
                //telemetry.addData("Team: ", "Blue 1");
            }
            else
            {
                // OPTION BLUE TWO (WHEELS)
                startDelay = 0;
                pivotAngle = -55;
                targetAngle = -90;
                startDist = 1270;
                targetIndex = 0;
                targetPos[0] = mmFTCFieldWidth;
                targetPos[1] = 1524;
                //telemetry.addData("Team: ", "Blue 2");
            }
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        VuforiaNav.startTracking();
        //     pause(startDelay);
        VuforiaNav.getLocation(targetIndex);

        //pause(delay);

        /*
        while (opModeIsActive()) {

            VuforiaNav.getLocation();
            telemetry.addData("tracking ", VuforiaNav.isVisible() ? "Visible" : "Not Visible");
            if (VuforiaNav.lastLocation != null)
            {
                telemetry.addData("location ", format(VuforiaNav.lastLocation));
            }

            telemetry.update();
            //pause(500);
        }
        */


// TESTS

        //TODO: test diagonal at 0 degrees
        //TODO: figure out how to use accelerometer
        //TODO: test with Vuforia
        //TODO: make sideways fast and go longer


/*
        forwards(30, 0, 0.7, 3); // make this in mm
        pause(200);
        forwards(0, 20, 0.7, 3);
        pause(30000);
        telemetry.addData("Path", "Done (end it!)");
*/

/*
        telemetry.addData("Path", "forwards, 0.5");
        telemetry.update();
        pivotMove(0, 300, 0, 0.5, 3);
        pause(3000);
*/



        // square stuff
        /*
        telemetry.addData("Path", "forwards, 0.8");
        telemetry.update();
        pivotMove(0, 1220, 0, 0.8, 3);
        pause(3000);

        telemetry.addData("Path", "right, 0.5");
        telemetry.update();
        pivotMove(1220, 0, 0, 0.5, 4);
        pause(3000);

        telemetry.addData("Path", "left, 0.5");
        telemetry.update();
        pivotMove(-1220, 0, 0, 0.5, 4);
        pause(3000);
        */

/*
        telemetry.addData("Path", "pivot left");
        telemetry.update();
        pivotMove(0, 0, 90, 0.8, 3);
        pause(3000);

        telemetry.addData("Path", "pivot right");
        telemetry.update();
        pivotMove(0, 0, -90, 0.8, 3);
*/
        //telemetry.addData("Path", "Done (end it!)");
        //telemetry.update();

/*
        TOL_ANGLE = 3;
        pivotDetectTarget(30, 5);
        TOL_ANGLE = 0.5;
        alignPivotVuforia(targetAngle, 600, 4);
        //pivotVuforia(0.5, 0.5);
        telemetry.addData("Path", "Done (end it!)");
        telemetry.update();
        pause(2000);
*/

        VUFORIA_TOL_ANGLE = 2;
        TOL = 40;
        TOL_ANGLE = 1;
        Kmove = 1.0/1500.0;
        Kpivot = 1.0/100.0;

        pivotMove(1220, 0, 0, 0.7, 4);
        pause(200);
        //pivotMove(0, 0, -90, 0.8, 4);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
