package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

import java.util.Locale;

/**
*  This is a test class for scoring the sampling field using OpenCV.
*/

@Autonomous(name="UsingOpenCV")

public class UsingOpenCV extends MasterAutonomous
{
    private OpenCVGold OpenCVVision;
    @Override
    public void runOpMode() throws InterruptedException
    {
        OpenCVVision = new OpenCVGold();

        OpenCVVision.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        OpenCVVision.setShowCountours(false);

        OpenCVVision.enable();

        initializeAuto();

        waitForStart();

        OpenCVVision.setShowCountours(true);

        //while (opModeIsActive())
        //{
//            //if it does not see gold rectangle
//            while(((OpenCVVision.getGoldRect().y + OpenCVVision.getGoldRect().height) / 2) <= 0)
//            {
//                //turn robot until it does
//                turnTo(currentAngle + 5); //not sure what turn angle I should do
//            }
            //gold is towards left of phone screen in horizontal  (rotated counter clockwise 90 degrees looking at it from the front)
            if(((OpenCVVision.getGoldRect().y + OpenCVVision.getGoldRect().height) / 2) <= Constants.GOLD_LEFT_HORIZONTAL)
            {
                //turnTo(35);
                moveRobot(90, 0.7, 0.25);
                moveRobot(135, 0.7, 1);
                telemetry.addLine("Left");
            }
            //gold is towards right of phone screen in horizontal position (rotated counter clockwise 90 degrees looking at it from the front)
            else if(((OpenCVVision.getGoldRect().y + OpenCVVision.getGoldRect().height) / 2) >Constants.GOLD_RIGHT_HORIZONTAL)
            {
                moveRobot(90, 0.7, 0.25);
                moveRobot(45, 0.7, 1);
                telemetry.addLine("Right");
            }
            //gold is towards middle of phone screen in horizontal position (rotated counter clockwise 90 degrees looking at it from the front)
            else
            {
                moveRobot(90, 0.7, 1.2);
                telemetry.addLine("Cannot tell");
            }

            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (OpenCVVision.getGoldRect().x + OpenCVVision.getGoldRect().width) / 2, (OpenCVVision.getGoldRect().y + OpenCVVision.getGoldRect().height) / 2));
            telemetry.addData("currentAngle", currentAngle);
            telemetry.update();


        //}

        // stop the vision system
        OpenCVVision.disable();

    }
}
