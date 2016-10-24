package org.firstinspires.ftc.team6220;

import com.qualcomm.hardware.adafruit.BNO055IMU;

/**
 * Created by Colew on 9/18/2016.
 */
abstract public class MasterAutonomous extends MasterOpMode
{
    //a function for finding the distance between two points
    public double FindDistance(double x1,double y1, double x2, double y2)
    {
        double Distance = Math.pow(Math.pow(x1-x2, 2) + Math.pow(y1-y2, 2), 0.5);

        return Distance;
    }

    //we use this in autonomous to drive to a specific point
    public void DriveTo(double x, double y)
    {
        //the starting position of the robot on the field (units in meters)
        double robotXPos = 0.2;
        double robotYPos = 2.4;

        while (FindDistance(robotXPos, robotYPos, x, y) != 0)
        {
            double xDiff = x - robotXPos;
            double yDiff = y - robotYPos;

            if (xDiff > 1)
            {
                xDiff = 1;
            }

            if (yDiff > 1)
            {
                yDiff = 1;
            }
            //y direction is reversed
            drive.moveRobot(xDiff/3, -yDiff, 0);

        }
    }


}
