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
        TOL = 100.0;
        TOL_ANGLE = 2.5;
        Kpivot = 1/100.0;

        double refAngle = imu.getAngularOrientation().firstAngle;

        // grab the glyph TODO: write methods to open the GG
        closeGG();
        /*
        moveTimed(0.4, 0, 1000);
        //move(300, 0, 0.1, 0.5, 5);
        sleep(200);
        pivotWithReference(0, refAngle, 0.1, 0.4);
        sleep(200);
        moveMaintainHeading(205, 0, 0, refAngle, 0.15, 0.6, 5);
        sleep(200);
        pivotWithReference(181, refAngle, 0.1, 0.55);
        move(0, -120, 0.1, 0.3, 3);
        */
        // TODO: write method to close the GG


        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
