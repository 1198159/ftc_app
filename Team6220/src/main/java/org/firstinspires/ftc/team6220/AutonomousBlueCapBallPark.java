package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
    Autonomous program used when only bumping the cap ball on the blue side
*/

@Autonomous(name = "AutoBlueCapBallPark", group = "Autonomous")
public class AutonomousBlueCapBallPark extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        //this is used to add absolute orientation to each autonomous program
        setHeadingOffset(90.0);

        drive.robotLocation = new Transform2D(2.395, 0.210, 90.0);

        waitForStart();

        drive.moveRobot(0.4, 1.0, 0.0);

        pause(1400);

        stopAllDriveMotors();
    }
}
