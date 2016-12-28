package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.external.samples.TemplateOpMode_Linear;

/**
 * Created by user on 12/12/2016.
 */

@Autonomous(name="Autonomous Tests", group = "Swerve")
// @Disabled

public class AutonomousTests extends MasterAutonomous
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
                pivotAngle = 50; // pivot this amount before acquiring target
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
                pivotAngle = 50; // pivot this amount before acquiring target
                targetAngle = 0; // Vuforia angle
                startDist = 1397;
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
                pivotAngle = -50; // recalc pivot?? also for red one??
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
                pivotAngle = -50;
                targetAngle = -90;
                startDist = 1397;
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
        VUFORIA_TOL_ANGLE = 2;
        TOL = 30;
        TOL_ANGLE = 2;
        Kmove = 1.0/1200.0;
        Kpivot = 1.0/140.0;


        telemetry.addData("Path", "start forwards");
        telemetry.update();
        // go towards target
        move(0, startDist, 0.7, 3);
        pause(100);

        telemetry.addData("Path", "pivot 60");
        telemetry.update();
        // pivot to face target
        pivot(pivotAngle, 0.7); // make sure IMU is on
        pause(200);

        telemetry.addData("Path", "scanning for target");
        telemetry.update();
        TOL_ANGLE = 3;
        pivotDetectTarget(30, 5);
        TOL_ANGLE = 0.5;
        telemetry.addData("Path", "align pivot vuf");
        telemetry.update();
        alignPivotVuforia(targetAngle, 600, 4);
        pause(100);

        do
        {
            VuforiaNav.getLocation(targetIndex); // update target location and angle
        }
        while (VuforiaNav.lastLocation == null);

// detect beacon color of left side: 0 is blue, 1 is red
        int beaconColor = VuforiaNav.GetBeaconColor();
        telemetry.log().add(String.format("LeftSide: %f, RightSide: %f", VuforiaNav.leftColorHSV[0], VuforiaNav.rightColorHSV[0]));
        telemetry.log().add(String.format("Returned Color: %d", beaconColor));
        if (isRedTeam)
        {
            telemetry.log().add(String.format("team red"));
        }
        else
        {
            telemetry.log().add(String.format("team blue"));
        }
        telemetry.update();

// shift left or right before pushing button
        if (beaconColor == 0)   // if left side beacon is blue
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                move(100, 0, 0.25, 3); // shift right
                PushButton();
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                move(-38, 0, 0.25, 4); // shift left
                PushButton();
            }
        }
        else if (beaconColor == 1)  // if left side beacon is red
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                move(-38, 0, 0.25, 4); // shift left
                PushButton();
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                move(100, 0, 0.25, 3); // shift right
                PushButton();
            }
        }
        else // when the color is unknown
        {
            telemetry.addData("Path", "unknown color, moving on");
            telemetry.update();
        }

        pause(100);
        move(0, -250, 0.25, 3); // back up from button (or just back up)

        // determine next beacon target
        if (isRedTeam) // if team RED
        {
            // OPTION RED ONE (TOOLS)
            targetIndex = 1;
            targetPos[0] = 2743.2f;
            targetPos[1] = mmFTCFieldWidth;
            //telemetry.addData("Team: ", "Red 1"); // display what team we're on after choosing with the buttons
        }
        else // if team BLUE
        {
            // OPTION BLUE ONE (LEGOS)
            targetIndex = 2;
            targetPos[0] = mmFTCFieldWidth;
            targetPos[1] = 2743.2f;
            //telemetry.addData("Team: ", "Blue 1");
        }

        Kmove = 1.0/2000.0;
        Kpivot = 1.0/50.0;
// shift to new target!!
        telemetry.addData("Path", "shift to new target");
        telemetry.update();
        if (beaconColor == 0) // if left side blue
        {
            if (isRedTeam) // move shorter
            {
                //forwards(0, 1220, 0.6, 4);
                pivotMove(1220, 0, 0, 0.7, 4);
            }
            else // move longer
            {
                //forwards(0, -1220, 0.6, 4);
                pivotMove(-1220, 0, 0, 0.7, 4);
            }
        }
        else if (beaconColor == 1) // if left side red
        {
            if (isRedTeam) // move longer
            {
                //forwards(0, 1220, 0.6, 4);
                pivotMove(1220, 0, 0, 0.7, 4);
            }
            else // move shorter
            {
                //forwards(0, -1220, 0.6, 4);
                pivotMove(-1220, 0, 0, 0.7, 4);
            }
        }
        pause(200);

        Kmove = 1.0/1200.0;
        Kpivot = 1.0/140.0;
        telemetry.addData("Path", "scanning for target");
        telemetry.update();
        TOL_ANGLE = 3;
        pivotDetectTarget(30, 5);
        TOL_ANGLE = 0.5;
        telemetry.addData("Path", "align pivot vuf");
        telemetry.update();
        alignPivotVuforia(targetAngle, 600, 4);
        pause(100);


        // detect beacon color of left side: 0 is blue, 1 is red
        beaconColor = VuforiaNav.GetBeaconColor();

        // shift left or right before pushing button
        if (beaconColor == 0)   // if left side beacon is blue
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                move(100, 0, 0.25, 3); // shift right
                PushButton();
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                move(-38, 0, 0.25, 4); // shift left
                PushButton();
            }
        }
        else if (beaconColor == 1)  // if left side beacon is red
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                move(-38, 0, 0.25, 4); // shift left
                PushButton();
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                move(100, 0, 0.25, 3); // shift right
                PushButton();
            }
        }
        else // when the color is unknown
        {
            telemetry.addData("Path", "unknown color, standing by");
            telemetry.update();
        }

        PushButton();
        pause(100);
        move(0, -300, 0.25, 3); // back up

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
