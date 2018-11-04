package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

import java.util.Locale;
import java.util.Random;

/**
 * This is a simple "hello world" opmode
 *
 */

@Autonomous(name="HelloWorld", group="Swerve")  // @Autonomous(...) is the other common choice
//@Disabled
public class HelloWorld extends MasterAutonomous
{
    private OpenCVDetect goldVision;
    boolean isLeftGold = false;
    boolean isCenterGold = false;
    boolean isRightGold = false;

    public void runOpMode() throws InterruptedException
    {

        autoInitializeRobot();
        waitForStart();

        autoInitializeRobot();
        goldVision = new OpenCVDetect();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        goldVision.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        goldVision.setShowCountours(false);
        // start the vision system
        goldVision.enable();
        telemetry.addData("Done: ", "initializing");
        telemetry.update();

        while (!isStarted())
        {
            // select position left or right, from drivers facing the field
            if (gamepad1.x) isPosCrater = true;
            if (gamepad1.b) isPosCrater = false;

            if (isPosCrater) telemetry.addData("Alliance: ", "Blue Crater");
            else telemetry.addData("Alliance: ", "Blue Depot");

            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2), (goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2))
            );

            if( ((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) <= 120) && ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) >= 140) )
            {
                //goldLocation = sampleFieldLocations.right;
                isLeftGold = false;
                isCenterGold = false;
                isRightGold = true;
                telemetry.addLine("Right");
            }
            else if( ((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) >= 140) && ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) >= 140))
            {
                //goldLocation = sampleFieldLocations.right;
                isLeftGold = false;
                isCenterGold = true;
                isRightGold = false;
                telemetry.addLine("Center");
            }
            else
            {
                isLeftGold = true;
                isCenterGold = false;
                isRightGold = false;
                telemetry.addLine("Left");
            }

            telemetry.update();
            idle();
        }
        telemetry.update();


        waitForStart();
        autoRuntime.reset();
        goldVision.disable();
        //autoRuntime.reset();
        telemetry.addData("Auto Blue: ", "Started");
        telemetry.update();

        // set the reference angle
        double refAngle = imu.getAngularOrientation().firstAngle; // possibly move to initialization

        pivotWithReference(45, refAngle, 0.2,0.5);

        /*
        land();
        moveTimed(0.2,200);
        pivotWithReference(0, refAngle, 0.2,0.5, 1);

        if (isLeftGold)
        {
            pivotWithReference(45, refAngle, 0.2, 0.75, 3);
        }
        else if (isRightGold)
        {
            pivotWithReference(-45, refAngle, 0.2, 0.75, 3);
        }

        sleep(100);
        moveTimed(0.55, 1000);
        sleep(100);
        //moveTimed(-0.55, 2000);

        if (!isPosCrater) // if depot position, then deposit team marker
        {
            if (isLeftGold)
            {
                pivotWithReference(45, refAngle, 0.2,0.75,3);
            }
            else if (isRightGold)
            {

            }
        }
        *
        */


    }
}
