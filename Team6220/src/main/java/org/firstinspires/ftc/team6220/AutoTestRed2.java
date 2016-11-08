package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This opmode is used to experiment with autonomous ideas before implementing them in our competition code.
 */

@Autonomous(name = "AutoTestRed2", group = "Autonomous")
public class AutoTestRed2 extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        initializeVuforia();

        drive.robotLocation = new Transform2D(0.380, 2.415, 0.0);

        waitForStart();

        vuforiaDriveToPosition(1.524, 3.656, 90);
    }
}
