package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Rect;

import java.util.Locale;


@Autonomous(name="OpenCVAutoTest", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class OpenCVAutoTest extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        openCVInit();

        openCVShowContours(true);

        waitForStart();

        openCVLocateGold();


        while (opModeIsActive())
        {
            telemetry.update();
            idle();
        }

    }
}
