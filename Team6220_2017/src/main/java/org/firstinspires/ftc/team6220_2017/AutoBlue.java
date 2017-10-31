package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Mridula on 10/29/2017.
 */
@Autonomous(name = "Auto Blue", group = "Autonomous")

public class AutoBlue extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        waitForStart();

        vuforiaHelper.getVumark();

        //if the vuMark is not visible, vuforia will tell us
        if (vuforiaHelper.isVisible())
        {
            boolean isLeftBlue = vuforiaHelper.getLeftJewelColor();
            boolean isBlueSide = true;
            telemetry.addData("leftColor ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("RightColor ", vuforiaHelper.avgRightJewelColor);
            knockJewel(isLeftBlue,isBlueSide);
        }
        else
        {
            //no matter what driveAngle I enter, it does not seem to register that and just moves straight (in the phone's direction)
            moveRobot(270, 1, 500);
            telemetry.addData("vuMark: ", "not visible");
        }


        telemetry.update();
    }

    //todo modify for jewels rather than beacons
    //we use this function to determine the color of jewels and knock them
    public void knockJewel (boolean isLeftBlue, boolean isBlueSide) throws InterruptedException
    {
        golfClubServo.setPosition(0.15);

        if(isBlueSide)
        {
            if(isLeftBlue)
            {
                turnTo(-90);
                moveRobot(90, 1, 500);
            }
            else
            {
                turnTo(90);
                moveRobot(-90, 1, 500);
            }
        }
        else
        {
            if(isLeftBlue)
            {
                turnTo(90);
                moveRobot(0, 1, 500);
            }
            else
            {
                turnTo(-90);
                moveRobot(180, 1, 500);
            }
        }
    }
    //moves the robot based on input
    public void moveRobot(double driveAngle, double drivePower, int pause) throws InterruptedException
    {
        driveMecanum(0, drivePower, 0.0);
        pause(pause);
        driveMecanum(0,0.0,0.0);
        //stopAllDriveMotors();

    }
}
