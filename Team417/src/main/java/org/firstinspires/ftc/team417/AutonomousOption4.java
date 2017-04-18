package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Option 4", group = "Swerve")
// @Disabled

public class AutonomousOption4 extends MasterAutonomous
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

// GO FOR, SHOOT, FOR AGAIN

        refAngle = imu.getAngularOrientation().firstAngle;

        Kpivot = 1.0/50.0;
        Kmove = 1.0/1500.0;
        TOL = 90;

        motorLauncher.setPower(0.85);
        //move(0, -800, 0.5 , 3);
        moveKeepHeading(0, -700, 0, refAngle, 0.8, 3);

        motorCollector.setPower(1.0);
        pause(1000);
        servoParticle.setPosition(SERVO_PARTICLE_HIGH);
        pause(300);
        servoParticle.setPosition(SERVO_PARTICLE_LOW);
        pause(1500);
        servoParticle.setPosition(SERVO_PARTICLE_HIGH);
        pause(300);
        servoParticle.setPosition(SERVO_PARTICLE_LOW);
        pause(300);
        motorLauncher.setPower(0.0);
        motorCollector.setPower(0.0);

        pause(10000);

        Kpivot = 1.0/120.0;
        TOL_ANGLE = 2.0;
        if (isRedTeam) pivot(55, 0.8);
        else pivot(-45, 0.8);

        refAngle = imu.getAngularOrientation().firstAngle;

        Kpivot = 1.0/50.0;
        //move(0, -1600, 0.7, 4);
        moveKeepHeading(0, -1700, 0, refAngle, 0.9, 3);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
