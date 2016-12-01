package org.firstinspires.ftc.team6220;

/**
 * Created by Colew on 9/18/2016.
 */
abstract public class MasterAutonomous extends MasterOpMode
{
    //used for initializations only necessary in autonomous
    public void initializeAuto()
    {
        initializeHardware();

        vuforiaHelper.setupVuforia();
    }

    //a function for finding the distance between two points
    public double findDistance(double x1, double y1, double x2, double y2)
    {
        double Distance = Math.pow(Math.pow(x1-x2, 2) + Math.pow(y1-y2, 2), 0.5);

        return Distance;
    }

    //uses vuforia to move to a location
    public void vuforiaDriveToPosition(double TargetX, double TargetY, double TargetAngle)
    {
        Transform2D TargetLocation = new Transform2D(TargetX, TargetY, TargetAngle);

        currentAngle = getAngularOrientationWithOffset();

        while((Math.abs(TargetX - drive.robotLocation.x) > Constants.xTolerance) || (Math.abs(TargetY - drive.robotLocation.y) > Constants.yTolerance) || (Math.abs(TargetAngle - drive.robotLocation.rot) > Constants.wTolerance))
        {
            float[] l = vuforiaHelper.getRobotLocation();
            //vuforia data comes out as an array instead of readable data, so it must be changed to a Transform2D;
            //also, vuforia data must be converted from millimeters to meters to be consistent with the rest of our code
            l[0] = l[0]/1000;
            l[1] = l[1]/1000;

            //we use this to convert our location from an array to a transform
            drive.robotLocation.SetPositionFromFloatArray(l);

            //use the imu to find our angle instead of vuforia; prevents wild rotation if vuforia does not locate target
            currentAngle = getAngularOrientationWithOffset();

            drive.robotLocation.rot = currentAngle;

            //Inform drivers of robot location. Location is null if we lose track of the targets
            if(vuforiaHelper.lastKnownLocation != null)
            {
                telemetry.addData("XPos: ", drive.robotLocation.x);
                telemetry.addData("YPos: ", drive.robotLocation.y);
                telemetry.addData("AngleDiff: ", TargetAngle - drive.robotLocation.rot);
                telemetry.update();
            }
            else
            {
                telemetry.addData("Pos:", "Unknown");

                telemetry.update();
            }

            //move the robot to the desired location
            double[] m = drive.navigateTo(TargetLocation);

            telemetry.addData("mX:", m[0]);
            telemetry.addData("mY:", m[1]);
            telemetry.addData("mW:", m[2]);

            idle();
        }
    }

    public void driveWhileTurning(double x, double y, double w)
    {
        double currentAngle = getAngularOrientationWithOffset();
        double angleDiff = w - currentAngle;
        double turningPower;

        //sets the power of the motors to turn.  Since the turning direction of the robot is reversed from the motors,
        //negative signs are necessary.  The extra added number is to make sure the robot does not slow down too
        //drastically when nearing its target angle.

        //CodeReview: please make the magic numbers be constants
        while(Math.abs(angleDiff) > Constants.minimumAngleDiff)
        {
            currentAngle = getAngularOrientationWithOffset();
            angleDiff = drive.normalizeRotationTarget(w, currentAngle);
            turningPower = angleDiff * Constants.turningPowerFactor;

            if (Math.abs(turningPower) > 1.0)
            {
                turningPower = Math.signum(turningPower);
            }

            // Make sure turn power doesn't go below minimum power
            if(turningPower > 0 && turningPower < Constants.turningPowerFactor)
            {
                turningPower = Constants.turningPowerFactor;
            }
            else if (turningPower < 0 && turningPower > -Constants.turningPowerFactor)
            {
                turningPower = -Constants.turningPowerFactor;
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

    //tells our robot to turn to a specified angle
    public void turnTo(double targetAngle)
    {
        double currentAngle = getAngularOrientationWithOffset();
        double angleDiff = drive.normalizeRotationTarget(targetAngle, currentAngle);
        double turningPower;

        //CodeReview: please make magic numbers be constants
        while(Math.abs(angleDiff) > Constants.minimumAngleDiff)
        {
            currentAngle = getAngularOrientationWithOffset();
            angleDiff = drive.normalizeRotationTarget(targetAngle, currentAngle);
            turningPower = angleDiff * Constants.turningPowerFactor;

            if (Math.abs(turningPower) > 1.0)
            {
                turningPower = Math.signum(turningPower);
            }

            // Makes sure turn power doesn't go below minimum power
            if(turningPower > 0 && turningPower < Constants.minimumTurningPower)
            {
                turningPower = Constants.minimumTurningPower;
            }
            else if (turningPower < 0 && turningPower > -Constants.minimumTurningPower)
            {
                turningPower = -Constants.minimumTurningPower;
            }
            else
            {

            }

            telemetry.addData("current angle: ", getAngularOrientationWithOffset());
            telemetry.update();

            drive.moveRobot(0.0, 0.0, -turningPower);

            idle();
        }

        stopAllDriveMotors();
    }
}
