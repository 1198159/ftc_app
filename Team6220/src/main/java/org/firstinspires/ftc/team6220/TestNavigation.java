package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Colew on 11/22/2016.
 */

@Autonomous(name = "TestNavigation", group = "Autonomous")
public class TestNavigation extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        initializeAuto();

        //this is used to add absolute orientation to each autonomous program
        setRobotStartingOrientation(90.0);

        drive.robotLocation = new Transform2D(1.500, 2.600, 90.0);

        waitForStart();

        //Start tracking targets
        vuforiaHelper.startTracking();

        vuforiaDriveToPosition(1.390, 3.000);
    }
}
