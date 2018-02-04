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

        double refAngle = imu.getAngularOrientation().firstAngle; // possibly move to initialization
        Kmove = 1.0/1200.0;
        TOL = 100.0;
        TOL_ANGLE = 2;
        Kpivot = 1/200.0;

        // lower the servos, putting jewel manipulator into position
        servoJewel.setPosition(JEWEL_LOW);
        sleep(200);

        pivotWithReference(17, refAngle, 0.12, 0.3); // then pivot left
        sleep(200);
        servoJewel.setPosition(JEWEL_INIT);
        sleep(200);
        pivotWithReference(0, refAngle, 0.15, 0.3); // then pivot back
        sleep(200);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();

    }
}
