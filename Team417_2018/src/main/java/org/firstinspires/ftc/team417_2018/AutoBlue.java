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

            if ( ((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) == 0) && ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) == 0) )
            {
                isLeftGold = true;
                isCenterGold = false;
                isRightGold = false;
                telemetry.addLine("Left");
            }
            else if( ((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) <= 160) /*&& ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) >= 470)*/ )
            {
                //goldLocation = sampleFieldLocations.right;
                isLeftGold = false;
                isCenterGold = false;
                isRightGold = true;
                telemetry.addLine("Right");
            }
            else if( ((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) >= 400) /*&& ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) >= 470)*/ )
            {
                //goldLocation = sampleFieldLocations.right;
                isLeftGold = false;
                isCenterGold = true;
                isRightGold = false;
                telemetry.addLine("Center");
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
            pivotWithReference(-40, refAngle, 0.2, 0.75);
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
            if (isLeftGold)
            {
                moveTimed(0.55, 1200); // push gold
                sleep(100);
                pivotWithReference(-20, refAngle, 0.2,0.75);
                sleep(100);
                moveTimed(0.55, 1300); // go into depot
                sleep(200);
                marker.setPosition(MARKER_HIGH); // drop the marker
            }
            else if (isRightGold)
            {
                moveTimed(0.55, 1200); // push gold
                sleep(100);
                pivotWithReference(14, refAngle, 0.2,0.75);
                moveTimed(0.55, 1200); // go into depot
                sleep(200);
                marker.setPosition(MARKER_HIGH); // drop the marker
                pivotWithReference(39, refAngle, 0.2,0.75); // face the crater
                move(0, -1000, 0.3, 0.7, 3.0);
            }
            else if (isCenterGold)
            {
                move(0, 700, 0.3, 0.7, 2.0); // push gold and go into crater
                sleep(200);
                move(50, 0, 0.2, 0.7, 1.0); // go closer to the wall
                marker.setPosition(MARKER_HIGH); // drop the marker
                pivotWithReference(42, refAngle, 0.2,0.75); // face the crater
                move(0, -1000, 0.3, 0.7, 3.0);
            }
            sleep(1000);
            lower();
            //moveTimed(-0.55, 2000); // move away from the crater

            //pivotWithReference(45, refAngle, 0.2,0.75); // turn so back of robot faces the crater
            //moveTimed(-0.55, 5500); // back up into the crater
        }
    }
}
