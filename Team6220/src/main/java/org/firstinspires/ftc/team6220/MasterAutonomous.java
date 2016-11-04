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

    public void VuforiaDriveToPosition(double TargetX, double TargetY, double TargetAngle )
    {
        //Start tracking targets
        vuforiaHelper.visionTargets.activate();

        vuforiaHelper.lastKnownLocation = vuforiaHelper.getLatestLocation();
        //updateLocation()

        //Inform drivers of robot location. Location is null if we lose track of targets
        if(vuforiaHelper.lastKnownLocation != null)
        {
            telemetry.addData("Pos", vuforiaHelper.format(vuforiaHelper.lastKnownLocation));
        }
        else
        {
            telemetry.addData("Pos", "Unknown");

            telemetry.update();
        }

    }

    //makes our robot turn to a specified angle
    public void TurnTo(double TargetAngle)
    {
        double currentAngle = imu.getAngularOrientation().firstAngle;
        double angleDiff = TargetAngle - currentAngle;
        double signedAddend;

        //the addend's sign depends on the sign of the angle difference.  If the angle difference is sufficiently small, we want
        //to remove the addend to stop the robot.
        if (angleDiff > 1.0)
        {
            signedAddend = 0.3;
        }
        else if (angleDiff < -1.0)
        {
            signedAddend = -0.3;
        }
        else
        {
            signedAddend = 0.0;
        }

        if (angleDiff > 0.7)
        {
            angleDiff = 0.7;
        }

        //sets the power of the motors to turn.  Since the turning direction of the robot is reversed from the motors,
        //negative signs are necessary.  The extra added number is to make sure the robot does not slow down too
        //drastically when nearing its target angle.
        driveAssemblies[FRONT_RIGHT].setPower(-angleDiff - signedAddend);
        driveAssemblies[FRONT_LEFT].setPower(-angleDiff - signedAddend);
        driveAssemblies[BACK_LEFT].setPower(-angleDiff - signedAddend);
        driveAssemblies[BACK_RIGHT].setPower(-angleDiff - signedAddend);
    }
}
