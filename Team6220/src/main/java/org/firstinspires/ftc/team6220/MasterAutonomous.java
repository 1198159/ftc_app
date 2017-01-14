package org.firstinspires.ftc.team6220;

import android.graphics.Color;
import android.support.annotation.BoolRes;

import java.util.concurrent.Delayed;

/*
    Contains important methods for use in our autonomous programs
*/
abstract public class MasterAutonomous extends MasterOpMode
{
    //variables used in autonomous setup
    Alliance alliance = Alliance.BLUE;
    RoutineOption routineOption = RoutineOption.LAUNCHANDBUTTONS;
    int delay = 0;

    //used for initializations only necessary in autonomous
    public void initializeAuto()
    {
        initializeHardware();
        vuforiaHelper = new VuforiaHelper();
        vuforiaHelper.setupVuforia();
    }

    //enums used for autonomous setup
    enum Alliance
    {
        BLUE,
        RED
    }
    enum RoutineOption
    {
        LAUNCHANDBUTTONS,
        LAUNCH,
        BUTTONS,
        PARKANDCAPBALL

    }

    //allows the driver to decide which autonomous routine should be run
    public void runSetUp()
    {
        //tells driver the routine options available
        telemetry.log().add("BlueSide/RedSide = X/B");
        telemetry.log().add("Delay/No = Y/A");
        telemetry.log().add("LaunchAndButtons = Dpad Left");
        telemetry.log().add("Launch = Dpad Right");
        telemetry.log().add("Buttons = Left Bumper");
        telemetry.log().add("ParkAndCapBall = Right Bumper");
        telemetry.log().add("Press start to exit setup.");

        boolean isSetUpRunning = true;

        while(isSetUpRunning)
        {
            if (driver1.isButtonPressed(Button.X))
            {
                alliance = Alliance.BLUE;
            }
            else if (driver1.isButtonPressed(Button.B))
            {
                alliance = Alliance.RED;
            }
            else if (driver1.isButtonPressed(Button.Y))
            {
                delay++;
            }
            else if (driver1.isButtonPressed(Button.A))
            {
                delay--;
            }
            else if (driver1.isButtonPressed(Button.DPAD_LEFT))
            {
                routineOption = RoutineOption.LAUNCHANDBUTTONS;
            }
            else if (driver1.isButtonPressed(Button.DPAD_RIGHT))
            {
                routineOption = RoutineOption.LAUNCHANDBUTTONS;
            }
            else if (driver1.isButtonPressed(Button.LEFT_BUMPER))
            {
                routineOption = RoutineOption.LAUNCHANDBUTTONS;
            }
            else if (driver1.isButtonPressed(Button.RIGHT_BUMPER))
            {
                routineOption = RoutineOption.LAUNCHANDBUTTONS;
            }
            else if (driver1.isButtonPressed(Button.START))
            {
                //stops the setup loop
                isSetUpRunning = false;
            }

            //ensures delay is not negative
            if (delay < 0)
            {
                delay = 0;
            }

            //displays current configuration
            telemetry.addData("Alliance: ", alliance.name());
            telemetry.addData("Delay: ", delay);
            telemetry.addData("Routine Option: ", routineOption.name());
            telemetry.update();

            idle();
        }

        telemetry.log().add("Setup finished.");
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

        //used to determine whether the robot has come near enough to its target location
        double positionOffsetMagnitude = Math.sqrt(Math.pow(TargetX - drive.robotLocation.x, 2) + Math.pow(TargetY - drive.robotLocation.y, 2));

        while (((positionOffsetMagnitude > Constants.POSITION_TOLERANCE) || (Math.abs(TargetAngle - drive.robotLocation.rot) > Constants.ANGLE_TOLERANCE)) && opModeIsActive())
        {
            positionOffsetMagnitude = Math.sqrt(Math.pow(TargetX - drive.robotLocation.x, 2) + Math.pow(TargetY - drive.robotLocation.y, 2));
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

            //removed for testing
            //Informs drivers of robot location. Location is null if we lose track of the targets
            if(vuforiaHelper.lastKnownLocation != null)
            {
                telemetry.addData("XPos: ", drive.robotLocation.x);
                telemetry.addData("YPos: ", drive.robotLocation.y);
                telemetry.update();
            }
            else
            {
                telemetry.addData("Pos:", "Unknown");
                telemetry.update();
            }

            //move the robot to the desired location
            double[] m = drive.NavigateTo(TargetLocation);

            telemetry.addData("XRate:", m[0]);
            telemetry.addData("YRate:", m[1]);
            telemetry.addData("WRate:", m[2]);
            telemetry.update();

            idle();
        }

        stopAllDriveMotors();
    }

    //uses vuforia to align with the center of the vision target; used because zig-zag pathing
    //in vuforiaDriveToPosition is inefficient
    public void vuforiaAlign(boolean redSide, boolean x, double targetPosition, double targetAngle)
    {
        double translationOffsetMagnitude;

        if (redSide)
        {
            //used to determine whether the robot has come near enough to its target location
            translationOffsetMagnitude = -(targetPosition - drive.robotLocation.x);
        }
        else
        {
            //used to determine whether the robot has come near enough to its target location
            translationOffsetMagnitude = targetPosition - drive.robotLocation.y;
        }

        drive.robotLocation.rot = getAngularOrientationWithOffset();

        while ((Math.abs(translationOffsetMagnitude) > Constants.POSITION_TOLERANCE /*|| Math.abs(targetAngle - drive.robotLocation.rot) > Constants.ANGLE_TOLERANCE*/) && opModeIsActive())
        {
            float[] l = vuforiaHelper.getRobotLocation();
            //vuforia data comes out as an array instead of readable data, so it must be changed to a Transform2D;
            //also, vuforia data must be converted from millimeters to meters to be consistent with the rest of our code
            l[0] = l[0]/1000;
            l[1] = l[1]/1000;

            //we use this to convert our location from an array to a transform
            drive.robotLocation.SetPositionFromFloatArray(l);

            drive.robotLocation.rot = getAngularOrientationWithOffset();

            if (redSide)
            {
                //used to determine whether the robot has come near enough to its target location
                translationOffsetMagnitude = -(targetPosition - drive.robotLocation.x);
            }
            else
            {
                //used to determine whether the robot has come near enough to its target location
                translationOffsetMagnitude = targetPosition - drive.robotLocation.y;
            }

            //Informs drivers of robot location. Location is null if we lose track of the targets
            if(vuforiaHelper.lastKnownLocation != null)
            {
                telemetry.addData("XPos: ", drive.robotLocation.x);
                telemetry.addData("YPos: ", drive.robotLocation.y);
            } else {
                telemetry.addData("Pos:", "Unknown");
            }

            //move the robot to the desired location
            drive.NavigateAxially(redSide, x, targetPosition, targetAngle);


            telemetry.addData("posRate:", translationOffsetMagnitude);
            telemetry.addData("wRate:", targetAngle - drive.robotLocation.rot);
            telemetry.addData("Position Offset: ", (Math.abs(translationOffsetMagnitude)));
            telemetry.update();

            idle();
        }

        stopAllDriveMotors();
    }

    //tells our robot to turn to a specified angle
    public void turnTo(boolean deadReckoning, double targetAngle)
    {
        //counter used to improve the accuracy of turnTo
        int  angleToleranceCounter = 0;

        if (deadReckoning)
        {
            currentAngle = getAngularOrientationWithOffset();
        }
        else
        {
            currentAngle = getRobotAngleUsingVuforia();
        }

        double angleDiff = normalizeRotationTarget(targetAngle, currentAngle);
        double turningPower;

        while(angleToleranceCounter < 50 && opModeIsActive())
        {
            //increases the counter if the robot's heading is in tolerance
            if (Math.abs(angleDiff) <= Constants.ANGLE_TOLERANCE)
            {
                angleToleranceCounter++;
            }

            if (deadReckoning)
            {
                currentAngle = getAngularOrientationWithOffset();
            }
            else
            {
                currentAngle = getRobotAngleUsingVuforia();
            }

            //gives robot its adjusted turning power
            angleDiff = normalizeRotationTarget(targetAngle, currentAngle);
            turningPower = angleDiff * Constants.TURNING_POWER_FACTOR;
            drive.RotationControlFilter.roll(turningPower);
            turningPower = drive.RotationControlFilter.getFilteredValue();

            //makes sure turn power doesn't go above maximum power
            if (Math.abs(turningPower) > 1.0)
            {
                turningPower = Math.signum(turningPower);
            }

            // Makes sure turn power doesn't go below minimum power
            if(turningPower > 0 && turningPower < Constants.MINIMUM_TURNING_POWER)
            {
                turningPower = Constants.MINIMUM_TURNING_POWER;
            }
            else if (turningPower < 0 && turningPower > -Constants.MINIMUM_TURNING_POWER)
            {
                turningPower = -Constants.MINIMUM_TURNING_POWER;
            }

            telemetry.addData("angleDiff: ", angleDiff);
            telemetry.update();

            drive.moveRobot(0.0, 0.0, -turningPower);

            idle();
        }

        stopAllDriveMotors();
    }

    //once at a beacon, we use this function to align with it
    public void AlignWithBeacon(boolean redSide, double position) throws InterruptedException
    {
        if (redSide)
        {
            pause(1000);

            turnTo(true, 90.0);

            float[] colorLeftSide = new float[]{0,0,0};
            float[] colorRightSide = new float[]{0,0,0};

            pause(1000);

            //Color.colorToHSV(vuforiaHelper.getPixelColor(-50, 185, 30), colorLeftSide);
            //Color.colorToHSV(vuforiaHelper.getPixelColor(50, 185, 30), colorRightSide);
            Color.colorToHSV(vuforiaHelper.getPixelColor(-127, 92, 0), colorLeftSide);
            Color.colorToHSV(vuforiaHelper.getPixelColor(127, 92, 0), colorRightSide);

            /*for(int i = 1; i <=40; i++)
            {
                for(int j = 1; j <= 40; j++)
                {
                    float[] tempColorLeft = new float[3];
                    Color.colorToHSV(vuforiaHelper.getPixelColor(-127 - j, 92 -i, 0), tempColorLeft);
                    Color.colorToHSV(vuforiaHelper.getPixelColor(127-j, 92-i, 0), colorRightSide);
                    int weight = i + j;
                    colorLeftSide[0] = ((colorLeftSide[0] * weight) + tempColorLeft[0]) / (weight + 1);
                }
            }*/

            //Red can be anywhere from 270 to 360 or 0 to 90.  Adding 360 ensures that the red side's
            //value is always greater than the blue side's, thus creating a positive value when blue is
            //subtracted from red and allowing the robot to drive to the correct side of the beacon
            if(colorLeftSide[0] < 90)
            {
                colorLeftSide[0] += 360;
            }
            if(colorRightSide[0] < 90)
            {
                colorRightSide[0] += 360;
            }

            //picks a side and navigates based on the color of the beacon
            if(colorLeftSide[0] - colorRightSide[0] < 0)
            {
                vuforiaAlign(true, true, position + Constants.BEACON_PRESS_OFFSET, 0.0);
            }
            else if(colorLeftSide[0] - colorRightSide[0] > 0)
            {
                vuforiaAlign(true, true, position - Constants.BEACON_PRESS_OFFSET, 0.0);
            }
            else
            {
                //if vuforia didn't find the color of the beacon, it tries again
                AlignWithBeacon(true, position);
            }

            stopAllDriveMotors();
        }
        else
        {
            pause(1000);

            turnTo(true, 0.0);

            float[] colorLeftSide = new float[3];
            float[] colorRightSide = new float[3];

            pause(1000);

            //Color.colorToHSV(vuforiaHelper.getPixelColor(-50, 185, 30), colorLeftSide);
            //Color.colorToHSV(vuforiaHelper.getPixelColor(50, 185, 30), colorRightSide);
            Color.colorToHSV(vuforiaHelper.getPixelColor(-127, 92, 0), colorLeftSide);
            Color.colorToHSV(vuforiaHelper.getPixelColor(127, 92, 0), colorRightSide);

            /*for(int i = 1; i <=40; i++)
            {
                for(int j = 1; j <= 40; j++)
                {
                    Color.colorToHSV(vuforiaHelper.getPixelColor(-127 - j, 92 -i, 0), colorLeftSide);
                    Color.colorToHSV(vuforiaHelper.getPixelColor(127-j, 92-i, 0), colorRightSide);
                }
            }*/

            //Red can be anywhere from 270 to 360 or 0 to 90.  Adding 360 ensures that the red side's
            //value is always greater than the blue side's, thus creating a positive value when blue is
            //subtracted from red and allowing the robot to drive to the correct side of the beacon
            if(colorLeftSide[0] < 90)
            {
                colorLeftSide[0] += 360;
            }
            if(colorRightSide[0] < 90)
            {
                colorRightSide[0] += 360;
            }

            //picks a side and navigates based on the color of the beacon
            if(colorLeftSide[0] - colorRightSide[0] < 0)
            {
                vuforiaAlign(false, true, position + Constants.BEACON_PRESS_OFFSET, 0.0);
            }
            else if(colorLeftSide[0] - colorRightSide[0] > 0)
            {
                vuforiaAlign(false, true, position - Constants.BEACON_PRESS_OFFSET, 0.0);
            }
            else
            {
                //if vuforia didn't find the color of the beacon, it tries again
                AlignWithBeacon(false, position);
            }

            stopAllDriveMotors();
        }
    }

    //gives the launcher time to update its state machine
    void pauseWhileUpdating(double time)
    {
        while(opModeIsActive() && time >= 0)
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();
            time -= eTime;

            telemetry.addData("eTime:", eTime);
            telemetry.addData("Time Remaining:", time);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }
}
