package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

import java.util.Locale;

@TeleOp(name="OpenCV Test")
public class OpenCVTest extends LinearOpMode
{
    private OpenCVDetect goldVision;
    @Override
    public void runOpMode()
    {
        goldVision = new OpenCVDetect();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        goldVision.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        goldVision.setShowCountours(false);
        // start the vision system
        goldVision.enable();

        waitForStart();
        goldVision.setShowCountours(true);

        while(opModeIsActive())
        {
            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (goldVision.getGoldRect().x + goldVision.getGoldRect().width) / 2, (goldVision.getGoldRect().y + goldVision.getGoldRect().height) / 2));
            telemetry.update();
        }

        goldVision.disable();
    }
}
