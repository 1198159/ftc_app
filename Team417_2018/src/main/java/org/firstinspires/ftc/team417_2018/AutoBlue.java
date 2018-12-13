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

    public void runOpMode() throws InterruptedException {
        autoInitializeRobot();

        goldVision = new OpenCVDetect();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        goldVision.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        goldVision.setShowCountours(false);
        // start the vision system
        goldVision.enable();
        telemetry.addData("Done: ", "initializing");
        telemetry.update();

        while (!isStarted()) {
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

        // sample
        telemetry.addData("Gold",
                String.format(Locale.getDefault(), "(%d, %d)", (goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2), (goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2))
        );

        if (((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) == 0) && ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) == 0)) {
            isLeftGold = false;
            isCenterGold = false;
            isRightGold = true;
            telemetry.addLine("Right");
        } else if (((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) <= 200) /*&& ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) >= 470)*/) {
            isLeftGold = true;
            isCenterGold = false;
            isRightGold = false;
            telemetry.addLine("Left");
        } else if (((goldVision.getGoldRect().y + goldVision.getGoldRect().height / 2) >= 400) /*&& ((goldVision.getGoldRect().x + goldVision.getGoldRect().width / 2) >= 470)*/) {
            isLeftGold = false;
            isCenterGold = true;
            isRightGold = false;
            telemetry.addLine("Center");
        }
        telemetry.update();
        idle();
        sleep(2000);
        // face the field

        // pivot to face left and right if needed

        if (!isPosCrater)
        {
            if (isRightGold)
            {
                pivotWithReference(55, refAngle, 0.2, 0.75); // pivot to face the gold mineral
                move(0, 550, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(500);
                pivotWithReference(135, refAngle, 0.2, 0.75); // turn to align
                sleep(500);
                move(0, 400, 0.3, 0.7, 3.0); // push the gold mineral
                sleep(500);
                pivotWithReference(-130, refAngle, 0.2, 0.75); // turn so pusher is facing the crater
                sleep(500);
                move(100, 0, 0.3, 0.7, 2.0); // push against the wall
                sleep(500);
                move(0, 3000, 0.3, 0.7, 3.0);// park in blue crater
            }
            else if (isLeftGold)
            {
                pivotWithReference(115, refAngle, 0.2, 0.75);
                sleep(100);
                move(0, 400, 0.3, 0.7, 2.0); // push the gold mineral
                pivotWithReference(45, refAngle, 0.2, 0.75); // turn so pusher is facing the crater
                move(0, 500, 0.3, 0.7, 3.0); // push the gold mineral
                pivotWithReference(50, refAngle, 0.2, 0.75); // turn to align
                move(-200, 0, 0.3, 0.7, 2.0); // push against the wall
                // move(2, 0, 0.3, 0.7, 2.0); // push against the wall
                move(0, -3000, 0.3, 0.7, 3.0);// park in blue crater
            }
            else if (isCenterGold)
            {
                pivotWithReference(87, refAngle, 0.2, 0.5); // turn to face the sampling field
                move(0, 800, 0.3, 0.7, 3.0); // push the gold mineral
                pivotWithReference(50, refAngle, 0.2, 0.75); // turn to align
                move(-180, 0, 0.3, 0.7, 2.0); // push against the wall
                // move(2, 0, 0.3, 0.7, 2.0); // push against the wall
                move(-8, -3000, 0.3, 0.7, 3.0);// park in blue crater
            }
        }

        if (isPosCrater) {
            if (isLeftGold)
            {

            }
            else if (isRightGold)
            {

            }

            moveTimed(0.55, 1200); // push gold
            sleep(100);
            pivotWithReference(0, refAngle, 0.2, 0.75); // face crater
            sleep(200);
            moveTimed(0.55, 500); // go into crater / park
        }

    }
    }

