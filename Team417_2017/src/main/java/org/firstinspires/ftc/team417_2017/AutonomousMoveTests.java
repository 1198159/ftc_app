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
        TOL = 60;
        TOL_ANGLE = 3;
        Kpivot = 1/140.0;
        PIVOT_MINSPEED = 0.15;

        double refAngle = imu.getAngularOrientation().firstAngle;
        pivotWithReference(-15, refAngle, 0.5); // then pivot right
        sleep(200);
        pivotWithReference(0, refAngle, 0.5); // then pivot back
        sleep(200);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
