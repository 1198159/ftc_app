package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *  Tests the driveStraight method
 */

@Autonomous(name = "TestDriveStraight", group = "Autonomous")
//@Disabled
public class TestDriveStraight extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //todo:  nullpointer exception
        //runSetUp();
        //initializeAuto();
        initializeHardware();

        //assumes blue alliance starting point
        drive.robotLocation = new Transform2D(2.395, 0.210, 90.0);
        setRobotStartingOrientation(90.0);

        waitForStart();

        //delay is in seconds; pause takes milliseconds
        pause(2000);

        ElapsedTime duration = new ElapsedTime();

        //Start tracking targets
        //vuforiaHelper.startTracking();
        double orientation = getAngularOrientationWithOffset();

        while (opModeIsActive() && (duration.seconds() < 5))
        {
            //TODO - need to set the x,y,w values appropriately
            drive.moveRobotAtConstantHeading(0.0, 0.15, 0.0, orientation);

            telemetry.addData("heading", getAngularOrientationWithOffset());
            telemetry.update();
            idle();
        }
        stopAllDriveMotors();
    }
}
