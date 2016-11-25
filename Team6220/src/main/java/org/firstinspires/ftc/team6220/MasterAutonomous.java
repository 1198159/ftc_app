package org.firstinspires.ftc.team6220;

import com.qualcomm.hardware.adafruit.BNO055IMU;

/**
 * Created by Colew on 9/18/2016.
 */
abstract public class MasterAutonomous extends MasterOpMode
{
    public double headingOffset;

    //used for initializations only necessary in autonomous
    public void initializeAuto()
    {
        initializeHardware();

        double headingOffset;

        vuforiaHelper.setupVuforia();
    }

    //a function for finding the distance between two points
    public double findDistance(double x1,double y1, double x2, double y2)
    {
        double Distance = Math.pow(Math.pow(x1-x2, 2) + Math.pow(y1-y2, 2), 0.5);

        return Distance;
    }

    //uses vuforia to move to a location
    public void vuforiaDriveToPosition(double TargetX, double TargetY, double TargetAngle)
    {
        double xTolerance = .010;
        double yTolerance = .010;
        double wTolerance = 3.0;

        Transform2D TargetLocation = new Transform2D(TargetX, TargetY, TargetAngle);

        while((Math.abs(TargetX - drive.robotLocation.x) > xTolerance) || (Math.abs(TargetY - drive.robotLocation.y) > yTolerance) || (Math.abs(TargetAngle - drive.robotLocation.rot) > wTolerance))
        {
            float[] l = vuforiaHelper.getRobotLocation();
            l[0] = l[0]/1000;
            l[1] = l[1]/1000;
            drive.robotLocation.SetPositionFromFloatArray(l);

            drive.robotLocation.rot = currentAngle + headingOffset;

            //Inform drivers of robot location. Location is null if we lose track of targets
            if(vuforiaHelper.lastKnownLocation != null)
            {
                telemetry.addData("PosX: ", drive.robotLocation.x);
                telemetry.addData("PosY: ", drive.robotLocation.y);
                telemetry.addData("AngleDiff: ", TargetAngle - drive.robotLocation.rot);
                telemetry.update();
            }
            else
            {
                telemetry.addData("Pos:", "Unknown");

                telemetry.update();
            }

            //move to the desired location
            double[] m = drive.navigateTo(TargetLocation);
            telemetry.addData("mX:", m[0]);
            telemetry.addData("mY:", m[1]);
            telemetry.addData("mW:", m[2]);
        }
        //turnTo(TargetAngle);
    }

    public void driveWhileTurning(double x, double y, double w)
    {
        double currentAngle = imu.getAngularOrientation().firstAngle + headingOffset;
        double angleDiff = w - currentAngle;
        double turningPower;

        //sets the power of the motors to turn.  Since the turning direction of the robot is reversed from the motors,
        //negative signs are necessary.  The extra added number is to make sure the robot does not slow down too
        //drastically when nearing its target angle.
        while(Math.abs(angleDiff) > 5.0)
        {
            currentAngle = imu.getAngularOrientation().firstAngle + headingOffset;
            angleDiff = drive.optimizeRotationTarget(w - currentAngle);
            turningPower = angleDiff / 1000;

            if (Math.abs(turningPower) > 1.0)
            {
                turningPower = Math.signum(turningPower);
            }

            // Make sure turn power doesn't go below minimum power
            if(turningPower > 0 && turningPower < 0.1)
            {
                turningPower = 0.1;
            }
            else if (turningPower < 0 && turningPower > -0.1)
            {
                turningPower = -0.1;
            }
            else
            {

            }

            telemetry.addData("angleDiff: ", angleDiff);
            telemetry.update();

            /*
            driveAssemblies[FRONT_RIGHT].setPower(-turningPower);
            driveAssemblies[FRONT_LEFT].setPower(-turningPower);
            driveAssemblies[BACK_LEFT].setPower(-turningPower);
            driveAssemblies[BACK_RIGHT].setPower(-turningPower);
            */

            drive.moveRobot(0.0, 0.0, -turningPower);

            idle();
        }

        stopAllDriveMotors();

    }

    //makes our robot turn to a specified angle
    public void turnTo(double TargetAngle)
    {
        double currentAngle = imu.getAngularOrientation().firstAngle;
        double angleDiff = TargetAngle - currentAngle;
        double turningPower;

        //sets the power of the motors to turn.  Since the turning direction of the robot is reversed from the motors,
        //negative signs are necessary.  The extra added number is to make sure the robot does not slow down too
        //drastically when nearing its target angle.
        while(Math.abs(angleDiff) > 5.0)
        {
            currentAngle = imu.getAngularOrientation().firstAngle;
            angleDiff = drive.optimizeRotationTarget(TargetAngle - currentAngle);
            turningPower = angleDiff / 1000;

            if (Math.abs(turningPower) > 1.0)
            {
                turningPower = Math.signum(turningPower);
            }

            // Make sure turn power doesn't go below minimum power
            if(turningPower > 0 && turningPower < 0.1)
            {
                turningPower = 0.1;
            }
            else if (turningPower < 0 && turningPower > -0.1)
            {
                turningPower = -0.1;
            }
            else
            {

            }

            telemetry.addData("angleDiff: ", angleDiff);
            telemetry.update();

            /*
            driveAssemblies[FRONT_RIGHT].setPower(-turningPower);
            driveAssemblies[FRONT_LEFT].setPower(-turningPower);
            driveAssemblies[BACK_LEFT].setPower(-turningPower);
            driveAssemblies[BACK_RIGHT].setPower(-turningPower);
            */

            drive.moveRobot(0.0, 0.0, -turningPower);

            idle();
        }

        stopAllDriveMotors();

    }
}
