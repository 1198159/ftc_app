package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;


@Autonomous(name="OpenCVAutoTest", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class OpenCVAutoTest extends MasterAutonomous
{
    OpenCV openCV = new OpenCV();
    @Override
    public void runOpMode() throws InterruptedException
    {
        openCV.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        openCV.setShowCountours(false);//wait for start to start program
        waitForStart();
        openCV.setShowCountours(true);

        if(((OpenCV.getGoldRect().y)) <= 150)
        {
            telemetry.addData("Position: ", "Left");
        }
        else if(((OpenCV.getGoldRect().y)) > 90)
        {
            telemetry.addData("Position: ", "Center");
        }
        else
        {
            telemetry.addData("Position", "Right");
        }
        telemetry.update();
        while (opModeIsActive())
        {
            idle();
        }

    }
}
