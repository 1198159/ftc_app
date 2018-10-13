package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

import java.util.Locale;

@Autonomous(name="UsingOpenCV")

public class UsingOpenCV extends MasterAutonomous
{
    private OpenCVGold OpenCVVision;
    @Override
    public void runOpMode()
    {
        OpenCVVision = new OpenCVGold();

        OpenCVVision.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        OpenCVVision.setShowCountours(false);

        OpenCVVision.enable();

        waitForStart();

        OpenCVVision.setShowCountours(true);

        while (opModeIsActive())
        {
            //if it does not see gold rectangle
            while(OpenCVVision.getGoldRect().x <= 0)
            {
                //turn robot until it does
                turnTo(currentAngle + 5); //not sure what turn angle I should do
            }

            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (OpenCVVision.getGoldRect().x + OpenCVVision.getGoldRect().width) / 2, (OpenCVVision.getGoldRect().y + OpenCVVision.getGoldRect().height) / 2));
            telemetry.addData("currentAngle", currentAngle);
            telemetry.update();


        }

        // stop the vision system
        OpenCVVision.disable();

    }
}
