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


// TODO: TESTS



        // square stuff
        /*
        telemetry.addData("Path", "forwards, 0.8");
        telemetry.update();
        pivotMove(0, 1220, 0, 0.8, 3);
        pause(3000);

        // for left and right
        TOL = 40;
        TOL_ANGLE = 1;
        Kmove = 1.0/2000.0;
        Kpivot = 1.0/50.0;

        telemetry.addData("Path", "right, 0.5");
        telemetry.update();
        pivotMove(1220, 0, 0, 0.65, 4);
        pause(10000);
        telemetry.addData("Path", "left, 0.5");
        telemetry.update();
        pivotMove(-1220, 0, 0, 0.65, 4);
        pause(300);
*/
        // align with Vuforia
/*
        TOL = 40;
        TOL_ANGLE = 3.0; // tol angle for scan is 3, not accurate
        Kmove = 1.0/1500.0;
        Kpivot = 1.0/100.0;
        telemetry.addData("Path", "scan");
        telemetry.update();
        pivotDetectTarget(30, 5);
        TOL_ANGLE = 2.0;
        VUFORIA_TOL_ANGLE = 2.0;
        telemetry.addData("Path", "alignPivotVuforia");
        telemetry.update();
        alignPivotVuforia(targetAngle, 600, 4);
        TOL_ANGLE = 1.0;
        VUFORIA_TOL_ANGLE = 1.0;
        telemetry.addData("Path", "pivotVuforia");
        telemetry.update();
        pivotVuforia(0.5, 0.5);
        telemetry.addData("Path", "Done (end it!)");
        telemetry.update();
        pause(2000);
*/

        /*
        VUFORIA_TOL_ANGLE = 2;
        TOL = 30;
        TOL_ANGLE = 2;
        Kmove = 1.0/1200.0;
        Kpivot = 1.0/140.0;

        startDelay = 0;
        pivotAngle = 60; // pivot this amount before acquiring target
        targetAngle = 0; // Vuforia angle
        startDist = 1397;
        targetIndex = 3;
        targetPos[0] = 1524;
        targetPos[1] = mmFTCFieldWidth;

        telemetry.addData("Path", "start forwards");
        telemetry.update();
        // go towards target
        move(0, startDist, 0.7, 3);
        pause(100);

        telemetry.addData("Path", "pivot 70");
        telemetry.update();
        // pivot to face target
        pivot(pivotAngle, 0.7); // make sure IMU is on
        pause(200);
*/
pause(100);
        // pivot and move tests
        TOL = 40;
        TOL_ANGLE = 1;
        Kmove = 1.0/1500.0;
        Kpivot = 1.0/100.0;

        pivotMove(1220, 1220, 0, 0.7, 4);
        pause(200);
        //pivotMove(0, 0, -90, 0.8, 4);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}