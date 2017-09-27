package org.firstinspires.ftc.team6220_2017;

import android.graphics.Color;

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

    //allows the driver to decide which autonomous routine should be run; not in use
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
        double Distance = Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));

        return Distance;
    }

    //todo needs to be changed
    //uses vuforia to move to a location
    public void vuforiaDriveToPosition(double targetX, double targetY)throws InterruptedException
    {

    }

    //tells the robot to turn to a specified angle
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

        //counter ensures that the robot is as close as possible to its target angle
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

            //todo needs to be changed
            /*
            drive.RotationControlFilter.roll(turningPower);
            turningPower = drive.RotationControlFilter.getFilteredValue();
            */

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

            //todo needs to be changed
            /*
            drive.moveRobot(0.0, 0.0, -turningPower);
            */

            idle();
        }

        stopAllDriveMotors();
    }

    //TODO: adjust pause times to allow vuforia just enough time to determine the jewel colors
    //we use this function to determine the color of jewels and knock them
    public void knockJewel (boolean redSide) throws InterruptedException
    {
        /*
        Color.colorToHSV(vuforiaHelper.getPixelColor(-127, 92, 0), colorLeftSide);
        Color.colorToHSV(vuforiaHelper.getPixelColor(127, 92, 0), colorRightSide);
        */
    }

    //gives the launcher time to update its state machine
    void pauseWhileUpdating(double time)
    {
        lTime = timer.seconds();

        while(opModeIsActive() && (time > 0) )
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
