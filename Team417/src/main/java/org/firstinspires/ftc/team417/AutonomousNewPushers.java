package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous New Pushers", group = "Swerve")
// @Disabled

public class AutonomousNewPushers extends MasterAutonomous
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
                startDist = 1397;
                targetIndex = 0;
                targetPos[0] = mmFTCFieldWidth;
                targetPos[1] = 1524;
                //telemetry.addData("Team: ", "Blue 2");
            }
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        autoRuntime.reset(); // set the 30 second timer

        VuforiaNav.startTracking();
        //     pause(startDelay);
        VuforiaNav.getLocation(targetIndex);

        //pause(delay);

// START OF AUTONOMOUS

        TOL_ANGLE = 3.0;
        VUFORIA_TOL_ANGLE = 3.0;
        TOL = 60;
        Kmove = 1.0/1200.0;
        Kpivot = 1/120.0;
        MINSPEED = 0.3;

        telemetry.addData("Path", "start forwards");
        telemetry.update();
        // go towards target
        moveAverage(0, startDist, 0, 0.7, 3);
        pause(100);

        PIVOT_MINSPEED = 0.2;
        telemetry.addData("Path", "pivot 70");
        telemetry.update();
        // pivot to face target
        pivot(pivotAngle, 0.9); // make sure IMU is on
        pause(200);

        TOL = 90;
        VUFORIA_TOL = 40;
        TOL_ANGLE = 3.0; // tol angle for scan is 3, not accurate
        VUFORIA_TOL_ANGLE = 3.0; // tol angle for scan is 3, not accurate
        Kmove = 1.0/1000.0;
        Kpivot = 1.0/150.0;
        MINSPEED = 0.3;

        telemetry.addData("Path", "scanning for first target");
        telemetry.update();
        pivotDetectTarget(30, 5);

        // setting for align pivot Vuforia
        TOL_ANGLE = 2.5;
        VUFORIA_TOL_ANGLE = 2.5;
        MINSPEED = 0.3;

        telemetry.addData("Path", "align pivot vuf");
        telemetry.update();
        alignPivotVuforia(0.6, 0, 600, 4);
        pause(50);

        // This reference angle is stored right before the robot pushes the button, so it's not out of alignment YET.
        // This angle will be later used for backing up with a reference to this angle.
        double refWallAngle = imu.getAngularOrientation().firstAngle;

        // TODO: Make this not sit forever if it can't find a target
        do
        {
            VuforiaNav.getLocation(targetIndex); // update target location and angle
        }
        while (VuforiaNav.lastLocation == null);

        // error came up here

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

        TOL_ANGLE = 3.0;
        VUFORIA_TOL_ANGLE = 3.0;
        TOL = 40;
        Kmove = 1.0/1200.0;
        MINSPEED = 0.15;

        if (beaconColor == 0)   // if left side beacon is blue
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                servoRightPusher.setPosition(RIGHT_PUSHER_HIGH);
                PushButton();
                pause(100);
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                servoLeftPusher.setPosition(LEFT_PUSHER_HIGH);
                PushButton();
                pause(100);
            }
        }
        else if (beaconColor == 1)  // if left side beacon is red
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                servoLeftPusher.setPosition(LEFT_PUSHER_HIGH);
                PushButton();
                pause(100);
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                servoRightPusher.setPosition(RIGHT_PUSHER_HIGH);
                PushButton();
                pause(100);
            }
        }
        else // when the color is unknown
        {
            telemetry.addData("Path", "unknown color, moving on");
            telemetry.update();
        }

        pause(100);

        TOL_ANGLE = 3.0;
        VUFORIA_TOL_ANGLE = 3.0;
        TOL = 60;
        Kmove = 1.0/1200.0;
        Kpivot = 1/140.0;
        MINSPEED = 0.35;
        moveMaintainHeading(0, -250, 0, refWallAngle, 0.5, 3); // back up
        // lower both of the pushers
        servoRightPusher.setPosition(RIGHT_PUSHER_LOW);
        servoLeftPusher.setPosition(LEFT_PUSHER_LOW);

        // determine next beacon target
        if (isRedTeam) // if team RED
        {
            // OPTION RED ONE (TOOLS)
            targetIndex = 1;
            targetPos[0] = 2743.2f;
            targetPos[1] = mmFTCFieldWidth;
        }
        else // if team BLUE
        {
            // OPTION BLUE ONE (LEGOS)
            targetIndex = 2;
            targetPos[0] = mmFTCFieldWidth;
            targetPos[1] = 2743.2f;
        }


        // for big move left or right
        TOL = 40;
        TOL_ANGLE = 1.5;
        Kmove = 1.0/2000.0;
        Kpivot = 1.0/120.0;
        MINSPEED = 0.3;
        PIVOT_MINSPEED = 0.15;
// shift to new target!!
        telemetry.addData("Path", "shift to new target");
        telemetry.update();

        if (beaconColor == 0) // if left side blue
        {
            if (isRedTeam) // move shorter
            {
                pivot(-90, 0.7);
                moveAverage(0, 1250, 0, 0.8, 3);
                pivot(90, 0.7);
            }
            else // move shorter
            {;
                pivot(90, 0.7);
                moveAverage(0, 1250, 0, 0.8, 3);
                pivot(-90, 0.7);
            }
        }
        else if (beaconColor == 1) // if left side red
        {
            if (isRedTeam) // move longer
            {
                pivot(-90, 0.7);
                moveAverage(0, 1250, 0, 0.8, 3);
                pivot(90, 0.7);
            }
            else // move longer
            {
                pivot(90, 0.7);
                moveAverage(0, 1250, 0, 0.8, 3);
                pivot(-90, 0.7);
            }
        }
        pause(200);

        TOL = 90;
        VUFORIA_TOL = 40;
        TOL_ANGLE = 3.0; // tol angle for scan is 3, not accurate
        VUFORIA_TOL_ANGLE = 3.0; // tol angle for scan is 3, not accurate
        Kmove = 1.0/1000.0;
        Kpivot = 1.0/150.0;
        MINSPEED = 0.3;

        telemetry.addData("Path", "scanning for target");
        telemetry.update();
        pivotDetectTarget(30, 5);

        TOL_ANGLE = 2.5;
        VUFORIA_TOL_ANGLE = 2.5;
        telemetry.addData("Path", "align pivot vuf");
        telemetry.update();
        alignPivotVuforia(0.6, 0, 600, 4);
        pause(50);

        // detect beacon color of left side: 0 is blue, 1 is red
        beaconColor = VuforiaNav.GetBeaconColor();

        // shift left or right before pushing button
        TOL_ANGLE = 3.0;
        VUFORIA_TOL_ANGLE = 3.0;
        TOL = 40;
        Kmove = 1.0/1200.0;
        Kpivot = 1/50.0;
        MINSPEED = 0.15;

        if (beaconColor == 0)   // if left side beacon is blue
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                servoRightPusher.setPosition(RIGHT_PUSHER_HIGH);
                PushButton();
                pause(100);
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                servoLeftPusher.setPosition(LEFT_PUSHER_HIGH);
                PushButton();
                pause(100);
            }
        }
        else if (beaconColor == 1)  // if left side beacon is red
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                servoLeftPusher.setPosition(LEFT_PUSHER_HIGH);
                PushButton();
                pause(100);
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                servoRightPusher.setPosition(RIGHT_PUSHER_HIGH);
                PushButton();
                pause(100);
            }
        }
        else // when the color is unknown
        {
            telemetry.addData("Path", "unknown color, moving on");
            telemetry.update();
        }
        pause(100);

        TOL_ANGLE = 3.0;
        VUFORIA_TOL_ANGLE = 3.0;
        TOL = 60;
        Kmove = 1.0/1200.0;
        Kpivot = 1/140.0;
        MINSPEED = 0.35;
        moveMaintainHeading(0, -250, 0, refWallAngle, 0.5, 3); // back up
        // lower both of the pushers
        servoRightPusher.setPosition(RIGHT_PUSHER_LOW);
        servoLeftPusher.setPosition(LEFT_PUSHER_LOW);

        if (autoRuntime.milliseconds() < 28000)
        {
            TOL_ANGLE = 3.0;
            VUFORIA_TOL_ANGLE = 3.0;
            Kmove = 1/1200.0;
            MINSPEED = 0.2;
            PIVOT_MINSPEED = 0.15;

            motorLauncher.setPower(0.85);

            if (isRedTeam) // RED
            {
                move(0, -100, 0.5, 3); // back up less
                pivot(-45, 0.8);
                move(0, -400, 0.6, 2); // come closer more
            }
            else // BLUE
            {
                move(0, -100, 0.5, 3); // back up less
                pivot(55, 0.8);
                move(0, -400, 0.6, 2); // come closer more
            }

            motorCollector.setPower(1.0);
            servoParticle.setPosition(0.8);
            pause(300);
            // Make sure time isn't up
            //if (autoRuntime.milliseconds() < 29000) return;
            servoParticle.setPosition(0.0);
            pause(1500);
            //if (autoRuntime.milliseconds() > 29500) return;
            servoParticle.setPosition(0.8);
            pause(300);
            servoParticle.setPosition(0.0);
            pause(300);
            motorLauncher.setPower(0.0);
            motorCollector.setPower(0.0);
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
