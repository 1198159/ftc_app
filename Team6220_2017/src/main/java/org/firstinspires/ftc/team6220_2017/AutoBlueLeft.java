package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Const;

/**
 * Created by Mridula on 10/29/2017.
 */
@Autonomous(name = "Auto Blue Left", group = "Autonomous")

public class AutoBlueLeft extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        waitForStart();

        vuforiaHelper.getVumark();
        boolean isBlueSide = true;

        //if the vuMark is not visible, vuforia will tell us
        if (vuforiaHelper.isVisible())
        {
            boolean isLeftBlue = vuforiaHelper.getLeftJewelColor();
            telemetry.addData("leftColor ", vuforiaHelper.avgLeftJewelColor);
            telemetry.addData("RightColor ", vuforiaHelper.avgRightJewelColor);
            knockJewel(isLeftBlue,isBlueSide);
        }
        else
        {
            moveRobot(270, 1, 1000);
            telemetry.addData("vuMark: ", "not visible");
        }

        telemetry.update();
    }

    //todo modify for jewels rather than beacons
    //we use this function to determine the color of jewels and knock them
    public void knockJewel (boolean isLeftBlue, boolean isBlueSide) throws InterruptedException
    {
        jewelJostlerServo.setPosition(Constants.JEWEL_JOSTLER_DEPLOYED);

        if(isBlueSide)
        {
            if(isLeftBlue)
            {
                turnTo(-90);
                moveRobot(0, 1, 1000);
            }
            else
            {
                turnTo(90);
                moveRobot(180, 1, 1000);
            }
        }
        else
        {
            if(isLeftBlue)
            {
                turnTo(-90);
                moveRobot(0, 1, 1000);
            }
            else
            {
                turnTo(90);
                moveRobot(180, 1, 1000);
            }
        }
    }
    //moves the robot based on input
    public void moveRobot(double driveAngle, double drivePower, int pause) throws InterruptedException
    {
        driveMecanum(driveAngle, drivePower, 0.0);
        pause(pause);
        driveMecanum(driveAngle,0.0,0.0);
        //stopAllDriveMotors();

    }
}
