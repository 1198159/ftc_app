package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Competition", group = "Swerve")
// @Disabled

public class AutonomousCompetition extends MasterAutonomous
{
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and other important things
        initializeRobot();

        VuforiaNav.initVuforia();
        telemetry.addData("Path", "Select Team and Pos...");

        // Wait until we're told to go
        while (!isStarted()) {
            // allow driver to choose a team
            if (gamepad1.b) {
                isRedTeam = true;
            }

            if (gamepad1.x) {
                isRedTeam = false;
            }

            // select position one or two, one is closer to the origin
            if (gamepad1.y) {
                isStartingPosOne = true;
            }
            if (gamepad1.a) {
                isStartingPosOne = false;
            }

            if (isRedTeam) {
                if (isStartingPosOne) {
                    telemetry.addData("Team: ", "Red 1");
                } else {
                    telemetry.addData("Team: ", "Red 2");
                }
            } else {
                if (isStartingPosOne) {
                    telemetry.addData("Team: ", "Blue 1");
                } else {
                    telemetry.addData("Team: ", "Blue 2");
                }
            }
            telemetry.update();
            idle();
        }
        telemetry.update();

        if (isRedTeam) // if team RED
        {
            if (isStartingPosOne) {
                // OPTION RED ONE (TOOLS)
                startDelay = 2000;
                pivotAngle = 55; // pivot this amount before acquiring target
                targetAngle = 0; // Vuforia angle
                startDist = 90;
                targetIndex = 1;
                targetPos[0] = 2743.2f;
                targetPos[1] = mmFTCFieldWidth;
                //telemetry.addData("Team: ", "Red 1"); // display what team we're on after choosing with the buttons
            } else {
                // OPTION RED TWO (GEARS)
                startDelay = 0;
                pivotAngle = 55; // pivot this amount before acquiring target
                targetAngle = 0; // Vuforia angle
                startDist = 50;
                targetIndex = 3;
                targetPos[0] = 1524;
                targetPos[1] = mmFTCFieldWidth;
                //telemetry.addData("Team: ", "Red 2"); // display what team we're on after choosing with the buttons
            }
            telemetry.update();
        } else // if team BLUE
        {
            if (isStartingPosOne) {
                // OPTION BLUE ONE (LEGOS)
                startDelay = 2000;
                pivotAngle = -55; // recalc pivot?? also for red one??
                targetAngle = -90;
                startDist = 90;
                targetIndex = 2;
                targetPos[0] = mmFTCFieldWidth;
                targetPos[1] = 2743.2f;
                //telemetry.addData("Team: ", "Blue 1");
            } else {
                // OPTION BLUE TWO (WHEELS)
                startDelay = 0;
                pivotAngle = -55;
                targetAngle = -90;
                startDist = 50;
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

// START OF AUTONOMOUS

        telemetry.addData("Path", "start forwards");
        telemetry.update();
        // go towards target
        //forwards(startDist, 0, 0.7, 3);  // inches, speed, timeout
        move(0, startDist, 0.7, 3);
        pause(100);

        telemetry.addData("Path", "pivot 60");
        telemetry.update();
        // pivot to face target
        pivot(pivotAngle, 0.7); // make sure IMU is on
        pause(100);

        telemetry.addData("Path", "align pivot vuf");
        telemetry.update();
        // align sideways with image
        alignPivotVuforia(0.7, 700, 3);
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
                //forwards(0, 2.5, 0.25, 3);   // shift right
                move(63.5, 0, 0.25, 3);
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                //forwards(0, -2.5, 0.25, 4);   // shift left
                move(-63.5, 0, 0.25, 3);
            }
        }
        else if (beaconColor == 1)  // if left side beacon is red
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                //forwards(0, -2.5, 0.25, 4);   // shift left
                move(-63.5, 0, 0.25, 3);
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                //forwards(0, 2.5, 0.25, 3);   // shift right
                move(63.5, 0, 0.25, 3);
                pause(100);
            }
        }
        else // when the color is unknown
        {
            telemetry.addData("Path", "unknown color, going back");
            telemetry.update();
            //forwards(-5, 0, 0.5, 3);
            move(0, 50, 0.5, 3);
        }

        //CodeReview: do you still try to push the button if the color is unknown?
        //            Or is this wasted movement because you backed up a moment ago?

        telemetry.addData("Path", "pushing button");
        telemetry.update();
        //forwards(17, 0, 0.25, 3); // push the button (first target)!!
        move(0, 431.8, 0.25, 3);
        telemetry.log().add(String.format("pushed first button"));
        pause(100);

        // back up and align once again
        telemetry.addData("Path", "back up and align");
        telemetry.update();
        //forwards(-20, 0, 0.3, 3);
        move(0, -508, 0.3, 3);
        pivotVuforia(targetAngle, 0.3);

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

// shift to new target!!
        telemetry.addData("Path", "shift to new target");
        telemetry.update();
        if (beaconColor == 0) // if left side blue
        {
            if (isRedTeam) // move shorter
            {
                //forwards(0, 36, 0.6, 4);
                move(914.4, 0, 0.6, 4);
            }
            else // move longer
            {
                //forwards(0, -965.2, 0.6, 4);
                move(-965.2, 0, 0.6, 4);
            }
        }
        else if (beaconColor == 1) // if left side red
        {
            if (isRedTeam) // move longer
            {
                //forwards(0, 38, 0.6, 4);
                move(965.2, 0, 0.6, 4);
            }
            else // move shorter
            {
                move(-914.4, 0, 0.6, 4);
            }
        }

        else // if color is unknown
        {
            if (isRedTeam) // move positive
            {
                //forwards(0, 35, 0.6, 4);
                move(889, 0, 0.6, 4);
            }
            else // move shorter
            {
                move(-889, 0, 0.6, 4);
            }
        }
        pause(1000);

        //CodeReview: you didn't handle the case where the beaconColor was unknown.
        //            Don't you still want to move to the next beacon?

        telemetry.addData("Path", "back up");
        telemetry.update();
        //forwards(-4, 0, 0.4, 2);
        move(0, 101.6, 0, 2);

        VuforiaNav.lastLocation = null;

        do
        {
            VuforiaNav.getLocation(targetIndex); // update target location and angle
        }
        while (VuforiaNav.lastLocation == null);

// align on new target
        telemetry.addData("Path", "align on new target");
        telemetry.update();
        alignPivotVuforia(0.7, 700, 3);

        // detect beacon color of left side: 0 is blue, 1 is red
        if (beaconColor == 0)   // if left side beacon is blue
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                //forwards(0, 2.5, 0.25, 3);   // shift right
                move(63.5, 0, 0.25, 3);
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                //forwards(0, -2.5, 0.25, 4);   // shift left
                move(-63.5, 0, 0.25, 3);
            }
        }
        else if (beaconColor == 1)  // if left side beacon is red
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                //forwards(0, -2.5, 0.25, 4);   // shift left
                move(-63.5, 0, 0.25, 3);
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                //forwards(0, 2.5, 0.25, 3);   // shift right
                move(63.5, 0, 0.25, 3);
                pause(100);
            }
        }
        else // when the color is unknown
        {
            telemetry.addData("Path", "unknown color, going back");
            telemetry.update();
            //forwards(-5, 0, 0.5, 3);
            move(0, 50, 0.5, 3);
        }

        telemetry.addData("Path", "align angle");
        telemetry.update();
        pivotVuforia(targetAngle, 0.3);

        telemetry.addData("Path", "push button");
        telemetry.update();
        move(0, 431.8, 0.25, 3);
        //forwards(17, 0, 0.25, 3); // push the button
        pause(100);
        move(0, -254, 0.25, 3);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        pause(10000);

    }
}
