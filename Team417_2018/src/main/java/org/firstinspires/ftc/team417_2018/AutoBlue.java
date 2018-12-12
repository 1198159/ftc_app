package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        //land();
        pivotWithReference(-90, refAngle, 0.2,0.5); // turn to face the sampling field
        move(0, -70, 0.3, 0.7, 2.0);
        // sample the gold mineral
        telemetry.addData("Gold",
                String.format(Locale.getDefault(), "(%d, %d)", (goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2), (goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2))
        );

        if ( ((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) == 0) && ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) == 0) )
        {
            isLeftGold = false;
            isCenterGold = false;
            isRightGold = true;
            telemetry.addLine("Right");
        }
        else if( ((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) <= 200) /*&& ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) >= 470)*/ )
        {
            isLeftGold = true;
            isCenterGold = false;
            isRightGold = false;
            telemetry.addLine("Left");
        }
        else if( ((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) >= 400) /*&& ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) >= 470)*/ )
        {
            isLeftGold = false;
            isCenterGold = true;
            isRightGold = false;
            telemetry.addLine("Center");
        }
        telemetry.update();
        idle();
        sleep(5000);
        // TODO: write translating when we get the arm motor working

        // pivot to face left and right
        if (isLeftGold)
        {
            pivotWithReference(50, refAngle, 0.2, 0.75);
        }
        else if (isRightGold)
        {
            pivotWithReference(-50, refAngle, 0.2, 0.75);
        }
        // if center, then don't pivot at all and just go forwards to push the mineral

        move(0, 300, 0.3, 0.7, 2.0); // push the gold mineral
        pivotWithReference(150, refAngle, 0.2, 0.75); // turn to align
        move(70, 0, 0.3, 0.7, 2.0); // push against the wall
        if (isPosCrater)
        {
            moveTimed(0.55, 1200); // push gold
            sleep(100);
            pivotWithReference(0, refAngle, 0.2, 0.75); // face crater
            sleep(200);
            moveTimed(0.55, 500); // go into crater / park
            lower();
        }
/*
        if (isPosCrater)
        {
            moveTimed(0.55, 1200); // push gold
            sleep(100);
            pivotWithReference(0, refAngle, 0.2, 0.75); // face crater
            sleep(200);
            moveTimed(0.55, 500); // go into crater / park
            lower();
        }


        if (!isPosCrater) // if depot position, then deposit team marker
        {
            if (isLeftGold)
            {
                sleep(100);
                pivotWithReference(-25, refAngle, 0.2,0.75);
                sleep(100);
                moveTimed(0.55, 1550); // go into depot
                sleep(200);
                marker.setPosition(MARKER_HIGH); // drop the marker
                pivotWithReference(45, refAngle, 0.2,0.75); // face the crater
                move(100, 0, 0.2, 0.7, 1.0); // go closer to the wall
                sleep(100);
                move(10, -1000, 0.3, 0.7, 3.0);
            }
            else if (isRightGold)
            {
                sleep(100);
                pivotWithReference(14, refAngle, 0.2,0.75);
                moveTimed(0.55, 1200); // go into depot
                sleep(200);
                marker.setPosition(MARKER_HIGH); // drop the marker
                pivotWithReference(41, refAngle, 0.2,0.75); // face the crater
                move(100, 0, 0.2, 0.7, 1.0); // go closer to the wall
                sleep(100);
                move(10, -1000, 0.3, 0.7, 3.0);
            }
            else if (isCenterGold)
            {
                sleep(200);
                move(50, 0, 0.2, 0.7, 1.0); // go closer to the wall
                marker.setPosition(MARKER_HIGH); // drop the marker
                pivotWithReference(42, refAngle, 0.2,0.75); // face the crater
                move(90, 0, 0.2, 0.7, 1.0); // go closer to the wall
                sleep(100);
                move(10, -1000, 0.3, 0.7, 3.0);
            }
            sleep(1000);

           // lower();
            //moveTimed(-0.55, 2000); // move away from the crater

            //pivotWithReference(45, refAngle, 0.2,0.75); // turn so back of robot faces the crater
            //moveTimed(-0.55, 5500); // back up into the crater
        }
        */
    }
}
