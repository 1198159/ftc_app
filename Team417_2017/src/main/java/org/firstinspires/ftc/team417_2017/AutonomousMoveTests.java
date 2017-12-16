package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Autonomous(name="Autonomous Move Tests", group = "Swerve")
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
        motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        closeGG();
        sleep(200);
        raiseGM();
        sleep(200);

        /*
        moveTimed(0.4, 0, 1000); // move off the balancing stone
        sleep(200);
        pivotWithReference(0, refAngle, 0.15, 0.4); // correct the robot's angle with ref. to the ref. angle
        sleep(200);
        */
        move(280, 0, 0.2, 0.4, 2.5); // move off the ramp
        sleep(200);
        pivotWithReference(0, refAngle, 0.15, 0.5);
        sleep(200);
        Kpivot = 1/70; // higher kPivot for his method because pivoting gets priority over encoder counts
        moveMaintainHeading(2, 0, 0, refAngle, 0.15, 0.6, 6);
        sleep(200);
        Kpivot = 1/100.0;
        pivotWithReference(181, refAngle, 0.15, 0.55);
        sleep(200);
        move(0, -200, 0.1, 0.3, 2.5); // push the glyph in
        sleep(200);

        motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        openGG(-500); // open the GG a little bit

        sleep(200);
        move(0, 150, 0.1, 0.3, 0.7); // back up from the cryptobox
        motorGlyphGrab.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        openGG(minGGPos);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();

    }
}
