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

        telemetry.addData("Path: ", "Init Done");
        telemetry.update();

// Wait for the game to start (driver presses PLAY)
        waitForStart();
        autoRuntime.reset(); // set the 30 second timer

// START OF AUTONOMOUS

        Kmove = 1.0/1200.0;
        MINSPEED = 0.15;
        TOL = 60;
        TOL_ANGLE = 3;
        Kpivot = 1/100.0;
        PIVOT_MINSPEED = 0.2;

        double refAngle = imu.getAngularOrientation().firstAngle;
        //move(500, 0, 0.6, 5);
        //sleep(400);
        //move(-1000, 0, 0.6, 5);
        //sleep(400);
        //pivotWithReference(180, refAngle, 0.3);
        moveMaintainHeading(0, 350, 0, refAngle, 0.6, PIVOT_MINSPEED, 5);
        sleep(5000);
        refAngle = imu.getAngularOrientation().firstAngle;
        moveMaintainHeading(350, 0, 0, refAngle, 0.6, PIVOT_MINSPEED, 5);
        sleep(400);
        refAngle = imu.getAngularOrientation().firstAngle;
        moveMaintainHeading(0, -350, 0, refAngle, 0.6, PIVOT_MINSPEED, 5);
        sleep(400);
        refAngle = imu.getAngularOrientation().firstAngle;
        moveMaintainHeading(-350, 0, 0, refAngle, 0.6, PIVOT_MINSPEED, 5);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
