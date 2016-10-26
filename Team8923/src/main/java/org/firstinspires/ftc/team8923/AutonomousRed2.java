package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 *  Runs an Autonomous program for the second position on the wall of the field that the
 *  red alliance drivers are stationed at.
 */
@Autonomous(name = "Auto Red 2", group = "Autonomous")
public class AutonomousRed2 extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();

        robotX = RED_2_START_X;
        robotY = RED_2_START_Y;
        robotAngle = RED_2_START_ANGLE;
        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;

        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();

        waitForStart();

        vuforiaLocator.startTracking();

        // Go in front of left beacon and give Vuforia a chance to start tracking the target
        goToLocation(1700, 3200, 90.0);
        sleep(1000);
        // TODO: Should the code below go into it's won method for pressing the beacon?
        // Go in front of left side of beacon
        goToLocation(1385, 3550, 90.0);
        sleep(500);
        // Check color of left side
        if(colorSensor.red() >= colorSensor.blue())
        {
            // Press left side if it's red
            telemetry.log().add("Left is red");
            goToLocation(1385, 3630, 90.0);
            sleep(500);
            goToLocation(1385, 3550, 90.0);
            sleep(500);
        }
        else
        {
            // Press right side if it's blue
            telemetry.log().add("Left is blue");
            goToLocation(1515, 3550, 90.0);
            sleep(500);
            goToLocation(1515, 3625, 90.0);
            sleep(500);
            goToLocation(1515, 3550, 90.0);
            sleep(500);
        }

        goToLocation(RED_2_START_X, RED_2_START_Y, RED_2_START_ANGLE);

        // TODO: Press other beacon button

        // TODO: Remove when testing is done. This is just so we can read the results
        sleep(5000);
    }
}
