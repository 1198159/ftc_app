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
        TOL_ANGLE = 2;
        Kpivot = 1/100.0;

        double refAngle = imu.getAngularOrientation().firstAngle;

        // grab the glyph
        closeGG();
        sleep(200);
        raiseGM();
        sleep(200);

        moveTimed(0.4, 0, 1000);
        sleep(200);
        pivotWithReference(0, refAngle, 0.1, 0.4);
        sleep(200);
        Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
        moveMaintainHeading(250, 0, 0, refAngle, 0.15, 0.6, 6);
        sleep(200);
        Kpivot = 1/100.0;
        pivotWithReference(181, refAngle, 0.1, 0.55);
        sleep(200);
        move(0, -200, 0.1, 0.3, 2.5); // push the glyph in // TODO: test for the right values
        sleep(200);
        // TODO: write something to open the glyph only a little bit
        move(0, 10, 0.1, 0.3, 0.7); // back up from the cryptobox

        openGG();
        sleep(200);

        move(0, 75, 0.1, 0.3, 0.7); // back up from the cryptobox

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();

    }
}
