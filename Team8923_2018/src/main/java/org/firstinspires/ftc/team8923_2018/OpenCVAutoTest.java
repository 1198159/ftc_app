package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;
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
        OpenCV openCV = new OpenCV();
        openCV.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        openCV.setShowCountours(false);//wait for start to start program
        openCV.enable();
        waitForStart();
        openCV.setShowCountours(true);

        Rect goldLocation = openCV.getGoldRect();

        if(((goldLocation.y + goldLocation.height / 2) < IMAGE_MIDDLE) && (goldLocation.y + goldLocation.height / 2) > 0)
        {
            telemetry.addData("Position: ", "Left");
        }
        else if((goldLocation.y + goldLocation.height / 2) >= IMAGE_MIDDLE)
        {
            telemetry.addData("Position: ", "Center");
        }
        else
        {
            telemetry.addData("Position", "Right");
        }


        while (opModeIsActive())
        {
            goldLocation = openCV.getGoldRect();
            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (goldLocation.x + goldLocation.width) / 2, (goldLocation.y + goldLocation.height) / 2));
            telemetry.update();

            idle();
        }

    }
}
