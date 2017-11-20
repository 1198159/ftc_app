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
        sleep(10);
        moveTimed(0.5, 0, 3000); // moe left with seconds
        sleep(100);
        // go left until the right sensor sees red; it should be on the right side of the triangle


        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
