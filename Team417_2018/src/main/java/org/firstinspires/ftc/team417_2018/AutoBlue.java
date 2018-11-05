package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

import java.util.Locale;

@Autonomous(name="Autonomous", group = "Swerve")
// @Disabled
public class AutoBlue extends MasterAutonomous
{
    private OpenCVDetect goldVision;
    boolean isLeftGold = false;
    boolean isCenterGold = false;
    boolean isRightGold = false;

    public void runOpMode() throws InterruptedException
    {
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

            if (isPosCrater) telemetry.addData("Alliance: ", "Crater");
            else telemetry.addData("Alliance: ", "Depot");

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
        telemetry.addData("Auto: ", "Started");
        telemetry.update();

        // set the reference angle
        double refAngle = imu.getAngularOrientation().firstAngle; // possibly move to initialization

        land();
        moveTimed(0.2,1200);
        pivotWithReference(0, refAngle, 0.2,0.5);

        // pivot to face left and right
        if (isLeftGold)
        {
            pivotWithReference(45, refAngle, 0.2, 0.75);
        }
        else if (isRightGold)
        {
            pivotWithReference(-45, refAngle, 0.2, 0.75);
        }

        if (isPosCrater)
        {
            moveTimed(0.55, 1200); // push gold
            sleep(100);
            pivotWithReference(0, refAngle, 0.2, 0.75); // face crater
            sleep(200);
            moveTimed(0.55, 500); // go into crater / park
        }


        //moveTimed(-0.55, 2000);

        if (!isPosCrater) // if depot position, then deposit team marker
        {
            moveTimed(0.55, 1200); // push gold
            sleep(100);
            if (isLeftGold)
            {
                pivotWithReference(-20, refAngle, 0.2,0.75);
                sleep(100);
                moveTimed(0.55, 1300); // go into depot
                sleep(200);
                marker.setPosition(MARKER_HIGH); // drop the marker
                //pivotWithReference(51, refAngle, 0.2,0.75);
                // forwards a little bit
                //moveTimed(-0.55, 750);
                // pivot so back of robot faces the crater
                //pivotWithReference(41, refAngle, 0.2,0.75);
            }
            else if (isRightGold)
            {
                pivotWithReference(10, refAngle, 0.2,0.75);
                moveTimed(0.55, 1200); // go into depot
                sleep(200);
                marker.setPosition(MARKER_HIGH); // drop the marker
            }
            else if (isCenterGold)
            {
                moveTimed(0.55, 500); // go into depot
                sleep(200);
                marker.setPosition(MARKER_HIGH); // drop the marker
            }
            sleep(1000);
            //moveTimed(-0.55, 2000); // move away from the crater

            //pivotWithReference(45, refAngle, 0.2,0.75); // turn so back of robot faces the crater
            //moveTimed(-0.55, 5500); // back up into the crater
        }
    }
}
