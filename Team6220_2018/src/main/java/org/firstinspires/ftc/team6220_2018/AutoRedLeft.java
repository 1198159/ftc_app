package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

import java.util.Locale;

@Autonomous(name="AutoRedLeft")

public class AutoRedLeft extends MasterAutonomous
{
    private OpenCVGold OpenCVVision;

    /*private enum sampleFieldLocations
    {
        left,
        center,
        right
    }

    AutoRedLeft.sampleFieldLocations goldLocation;*/

    @Override
    public void runOpMode() throws InterruptedException
    {
        OpenCVVision = new OpenCVGold();

        OpenCVVision.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        OpenCVVision.setShowCountours(false);

        OpenCVVision.enable();

        initializeAuto();

        OpenCVVision.setShowCountours(true);
        waitForStart();

        while(opModeIsActive())
        {
            // gold is towards left of phone screen in horizontal  (rotated counter clockwise 90 degrees looking at it from the front)
            if (((OpenCVVision.getGoldRect().y + (OpenCVVision.getGoldRect().height / 2)) <= Constants.GOLD_LEFT_HORIZONTAL))
            {
                //goldLocation = AutoRedLeft.sampleFieldLocations.left;
                /*moveRobot(90, 0.7, 0.25);
                moveRobot(135, 0.7, 1);
                turnTo(-45);
                moveRobot(180, 0.7, 0.1);
                moveRobot(-90, 0.7, 0.1);*/
                telemetry.addLine("Left");
            }
            // gold is towards right of phone screen in horizontal position (rotated counter clockwise 90 degrees looking at it from the front)
            else if (((OpenCVVision.getGoldRect().y + (OpenCVVision.getGoldRect().height / 2))) > Constants.GOLD_RIGHT_HORIZONTAL)
            {
                //goldLocation = AutoRedLeft.sampleFieldLocations.right;
                /*moveRobot(90, 0.7, 0.25);
                moveRobot(45, 0.7, 1);
                turnTo(-45);*/
                telemetry.addLine("Right");
            }
            // gold is towards middle of phone screen in horizontal position (rotated counter clockwise 90 degrees looking at it from the front)
            else
            {
                /*moveRobot(90, 0.7, 1.2);
                turnTo(-45);*/
                //goldLocation = AutoRedLeft.sampleFieldLocations.center;
                telemetry.addLine("Center (default)");
            }
            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (OpenCVVision.getGoldRect().x + (OpenCVVision.getGoldRect().width) / 2), (OpenCVVision.getGoldRect().y + (OpenCVVision.getGoldRect().height / 2))));
            telemetry.addData("currentAngle", currentAngle);
            telemetry.update();
        }


        //while (opModeIsActive())
        //{
//            //if it does not see gold rectangle
//            while(((OpenCVVision.getGoldRect().y + OpenCVVision.getGoldRect().height) / 2) <= 0)
//            {
//                //turn robot until it does
//                turnTo(currentAngle + 5); //not sure what turn angle I should do
//            }
        // gold is towards left of phone screen in horizontal  (rotated counter clockwise 90 degrees looking at it from the front)
        /*if(goldLocation == AutoRedLeft.sampleFieldLocations.left)
        {
            //turnTo(35);
            moveRobot(90, 0.7, 0.25);
            moveRobot(135, 0.7, 1);
            turnTo(-45);
            moveRobot(180, 0.7, 0.1);
            moveRobot(-90, 0.7, 0.1);
        }
        // gold is towards right of phone screen in horizontal position (rotated counter clockwise 90 degrees looking at it from the front)
        else if(goldLocation == AutoRedLeft.sampleFieldLocations.right)
        {
            moveRobot(90, 0.7, 0.25);
            moveRobot(45, 0.7, 1);
            turnTo(-45);

        }
        // gold is towards middle of phone screen in horizontal position (rotated counter clockwise 90 degrees looking at it from the front)
        else
        {
            moveRobot(90, 0.7, 1.2);
            turnTo(-45);

        }*/

        //}

        // stop the vision system
        OpenCVVision.disable();

    }
}
