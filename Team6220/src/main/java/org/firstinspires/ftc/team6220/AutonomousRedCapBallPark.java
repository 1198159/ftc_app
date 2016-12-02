package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
 Autonomous program used when only bumping the cap ball on the red side
*/

@Autonomous(name = "AutoRedCapBallPark", group = "Autonomous")
public class AutonomousRedCapBallPark extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        setHeadingOffset(0.0);

        drive.robotLocation = new Transform2D(0.210, 2.395, 0.0);

        waitForStart();

        //TODO test turning, then uncomment lower code
        turnTo(90.0);

        /*drive.moveRobot(-0.4, 1.0, 0.0);

        pause(1400);

        stopAllDriveMotors();*/
    }
}
