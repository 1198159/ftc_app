package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

import java.util.Locale;


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
        openCV.enable();
        waitForStart();
        openCV.setShowCountours(true);

        if(((OpenCV.getGoldRect().y + OpenCV.getGoldRect().height / 2) < 150) && (OpenCV.getGoldRect().y + OpenCV.getGoldRect().height / 2) > 0
                )
        {
            telemetry.addData("Position: ", "Left");
        }
        else if((OpenCV.getGoldRect().y + OpenCV.getGoldRect().height / 2) > 250)
        {
            telemetry.addData("Position: ", "Center");
        }
        else
        {
            telemetry.addData("Position", "Right");
        }
        telemetry.addData("Gold",
                String.format(Locale.getDefault(), "(%d, %d)", (OpenCV.getGoldRect().x + OpenCV.getGoldRect().width) / 2, (OpenCV.getGoldRect().y + OpenCV.getGoldRect().height) / 2));
        telemetry.update();
        while (opModeIsActive())
        {
            idle();
        }

    }
}
