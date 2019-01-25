package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;
import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.Locale;

@Autonomous(name="Hang Autonomous", group = "Swerve")
// @Disabled
public class AutoHang extends MasterAutonomous
{
    int threshold = 90;
    public void runOpMode() throws InterruptedException
    {
        autoInitializeRobot();
        InitializeDetection();

        while (!isStarted())
        {
            // select position left or right, from drivers facing the field
            if (gamepad1.x) isPosCrater = true;
            if (gamepad1.b) isPosCrater = false;

            if (gamepad1.y) threshold = 80;
            if (gamepad1.a) threshold = 90;
            OpenCV_detector.setThreshold(threshold);
            telemetry.addData("threshold", threshold);

            if (isPosCrater) telemetry.addData("Alliance: ", "CRATER");
            else telemetry.addData("Alliance: ", "DEPOT");

            telemetry.addData("Gold",
                    String.format(Locale.getDefault(), "(%d, %d)", (OpenCV_detector.getGoldRect().x + OpenCV_detector.getGoldRect().width / 2), (OpenCV_detector.getGoldRect().y + OpenCV_detector.getGoldRect().height / 2)));
            locateGold();

            telemetry.update();
        }
        waitForStart();
        autoRuntime.reset();
        telemetry.addData("Auto: ", "Started");

        land();
        // set the reference angle
        double refAngle = imu.getAngularOrientation().firstAngle; // possibly move to initialization

        move(80, 0, 0.2, 0.75, 3.0); // move robot hanger hanger off the lander hook
        sleep(50);

        move(0, 70, 0.2, 0.75, 3.0); // move from sampling position to gold push position
        sleep(50);

        if (!isPosCrater) // if it's the depot option
        {
            if (isLeftGold)
            {
                pivotWithReference(30, refAngle, 0.2, 0.75); // pivot to face the gold mineral
                sleep(100); // pause 100 milliseconds
                move(0, 500, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(100); // pause 100 milliseconds
                pivotWithReference(-45, refAngle, 0.2, 0.75); // turn to face depot
                sleep(100); // pause 100 milliseconds
                move(0, 400, 0.3, 0.7, 3.0); // go towards depot
                sleep(100);
                marker.setPosition(MARKER_HIGH);
                sleep(100);
                pivotWithReference(-45, refAngle, 0.2, 0.75); // turn to face depot
                sleep(100);
                move(-170, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(100);
                move(0, -2700, 0.3, 0.7, 3.0);// park in blue crater
                marker.setPosition(MARKER_LOW);
            }
            else if (isRightGold)
            {
                pivotWithReference(-30, refAngle, 0.2, 0.75); // pivot to face gold mineral
                sleep(100);
                move(0, 500, 0.3, 0.7, 2.0); // push the gold mineral
                sleep(100);
                pivotWithReference(-45, refAngle, 0.2, 0.75); // turn so pusher is facing the crater
                sleep(100);
                move(0, 500, 0.3, 0.7, 3.0); // go into depot
                sleep(100);
                //marker.setPosition(MARKER_HIGH);
                pivotWithReference(-40, refAngle, 0.2, 0.75); // turn to align
                sleep(100);
                move(-200, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(100);
                move(0, -2550, 0.3, 0.7, 3.0);// park in blue crater
                //marker.setPosition(MARKER_LOW);
            }
            else if (isCenterGold)
            {
                move(0, 800, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(100);
                pivotWithReference(-40, refAngle, 0.2, 0.75); // turn to align
                sleep(100);
                //marker.setPosition(MARKER_HIGH);
                move(-180, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(100);
                move(-8, -2600, 0.3, 0.7, 3.0);// park in blue crater
                //marker.setPosition(MARKER_LOW);
            }
        }

        if (isPosCrater)
        {
            if (isRightGold)
            {
                pivotWithReference(-25, refAngle, 0.2, 0.75); // pivot to face gold mineral
                sleep(100);
                move(0, 500, 0.3, 0.7, 2.0); // push the gold mineral
            }
            else if (isLeftGold)
            {
                pivotWithReference(30, refAngle, 0.2, 0.75); // pivot to face the gold mineral
                sleep(100);
                move(0, 500, 0.3, 0.7, 3.0); // push the gold mineral
            }
            else if (isCenterGold)
            {
                sleep(100);
                move(0, 500, 0.3, 0.7, 3.0); // push the gold mineral
            }

            sleep(100);
            pivotWithReference(0, refAngle, 0.2, 0.75); // face crater
            sleep(200);
            moveTimed(0.55, 500); // go into crater / park
        }

        vuforia.stop();
        }
    }
