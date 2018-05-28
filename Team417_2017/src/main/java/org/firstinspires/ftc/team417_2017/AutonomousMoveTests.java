package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        double refAngle = imu.getAngularOrientation().firstAngle;
        TOL_ANGLE = 2.0;
        /*
        move(0, -250, 0.8, 0.95, 0.5);
        sleep(50);
        move(0, 85, 0.7, 0.9, 0.5);*/
        deployGGExtensions();
        pivotWithReference(0, refAngle, 0.15, 0.3);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();

    }
}
