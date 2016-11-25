package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Colew on 11/20/2016.
 */

@Autonomous(name = "AutoRedCapBallPark", group = "Autonomous")
public class AutonomousRedCapBallPark extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        //this is used to add absolute orientation to each autonomous program
        headingOffset = 0.0;

        drive.robotLocation = new Transform2D(0.210, 2.395, 0.0 + headingOffset);

        waitForStart();

        //just in case the code above does not work
        drive.moveRobot(-0.4, 1.0, 0.0);
        pause(1400);
        stopAllDriveMotors();
    }
}
