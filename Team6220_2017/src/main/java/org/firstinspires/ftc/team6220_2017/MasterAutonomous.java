package org.firstinspires.ftc.team6220_2017;

import android.graphics.Color;

import java.util.ArrayList;

/*
    Contains important methods for use in our autonomous programs
*/
abstract public class MasterAutonomous extends MasterOpMode
{
    //variables used in autonomous setup
    Alliance alliance = Alliance.BLUE;
    /*
    RoutineOption routineOption = RoutineOption.;
    */
    int delay = 0;
    //use for more advanced auto
    ArrayList<Alliance> routine = new ArrayList<>();

    //enums used for runSetUp
    enum Alliance
    {
        BLUE,
        RED
    }

    public boolean isBlueSide = true;
    //used for initializations only necessary in autonomous
    public void initializeAuto()
    {
        /*boolean settingUp = true;

        while(settingUp)
        {
            if(gamepad1.left_bumper)
            {
                isBlueSide = false;
                telemetry.addLine("We are on red side!");
                telemetry.update();
            }
            else if(gamepad1.right_bumper)
            {
                isBlueSide = true;
                telemetry.addLine("We are on blue side!");
                telemetry.update();
            }
            else if(gamepad1.start)
                settingUp = false;

            idle();
        }*/
        initializeHardware();
        vuforiaHelper = new VuforiaHelper();
        vuforiaHelper.setupVuforia();
        //temporary
        /*routine.add(Alliance.BLUE);
        telemetry.addLine("We are on blue side!");
        telemetry.update();*/
    }

    //todo change
    enum RoutineOption
    {
        BEACON_LEFT,
        BEACON_RIGHT,
        PARK_RAMP,
        PARK_CENTER,
        SHOOT_CENTER
    }


    //todo write setup for this year's autonomous
    //note: not currently in use
    //allows the driver to decide which autonomous routine should be run; not in use
    public void runSetUp()
    {


        telemetry.log().add("Setup finished.");
    }

    //a function for finding the distance between two points
    public double findDistance(double x1, double y1, double x2, double y2)
    {
        double Distance = Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));

        return Distance;
    }

    //todo needs to be changed for mecanum
    //use vuforia to move to a location
    public void vuforiaDriveToPosition(double targetX, double targetY)throws InterruptedException
    {

    }

    //tell the robot to turn to a specified angle
    public void turnTo(double targetAngle)
    {
        double angleDiff = normalizeRotationTarget(targetAngle, currentAngle);
        double turningPower;

        //robot only stops when it is within angle tolerance
        while(Math.abs(angleDiff) >= Constants.ANGLE_TOLERANCE && opModeIsActive())
        {
            currentAngle = getAngularOrientationWithOffset();

            //gives robot its adjusted turning power
            angleDiff = normalizeRotationTarget(targetAngle, currentAngle);
            turningPower = Constants.TURNING_POWER_FACTOR * angleDiff;

            //sends turningPower through PID filter to increase precision of turning
            RotationControlFilter.roll(turningPower);
            turningPower = RotationControlFilter.getFilteredValue();

            //makes sure turn power doesn't go above maximum power
            if (Math.abs(turningPower) > 1.0)
            {
                turningPower = Math.signum(turningPower);
            }

            //makes sure turn power doesn't go below minimum power
            if(turningPower > 0 && turningPower < Constants.MINIMUM_TURNING_POWER)
            {
                turningPower = Constants.MINIMUM_TURNING_POWER;
            }
            else if (turningPower < 0 && turningPower > -Constants.MINIMUM_TURNING_POWER)
            {
                turningPower = -Constants.MINIMUM_TURNING_POWER;
            }

            //turns robot
            driveMecanum(0.0, 0.0, -turningPower);

            telemetry.addData("angleDiff: ", angleDiff);
            telemetry.update();

            idle();
        }

        stopAllDriveMotors();
    }

    //todo modify for jewels rather than beacons
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
    //wait a number of milliseconds
    public void pause(int t) throws InterruptedException
    {
        //we don't use System.currentTimeMillis() because it can be inconsistent
        long initialTime = System.nanoTime();
        while((System.nanoTime() - initialTime)/1000/1000 < t)
        {
            idle();
        }
    }
}
