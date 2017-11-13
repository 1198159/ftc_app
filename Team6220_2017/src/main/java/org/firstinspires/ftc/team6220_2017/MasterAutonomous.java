package org.firstinspires.ftc.team6220_2017;

import android.graphics.Color;

import java.util.ArrayList;

/*
    Contains important methods for use in our autonomous programs
*/
abstract public class MasterAutonomous extends MasterOpMode
{
    public boolean isBlueSide = true;

    //todo redundant blue side declarations
    // variables used in autonomous setup
    Alliance alliance = Alliance.BLUE;
    /*
    RoutineOption routineOption = RoutineOption.;
    */
    int delay = 0;
    // use for more advanced auto
    ArrayList<Alliance> routine = new ArrayList<>();

    // enums used for runSetUp-------------
    enum Alliance
    {
        BLUE,
        RED
    }
    /*
     note:  position of balancing stone is relative to where the driver stands (which changes based
     on alliance), so left and right are actually in opposite corners of the field
    */
    enum BalancingStone
    {
        LEFT,
        RIGHT
    }
    // ------------------------------------

    // used for object initializations only necessary in autonomous
    public void initializeAuto()
    {
        // we don't want to run the arm during autonomous
        isArmAttached = false;
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
        }

        // temporary
        routine.add(Alliance.BLUE);
        telemetry.addLine("We are on blue side!");
        telemetry.update();
        */

        initializeHardware();
        vuforiaHelper = new VuforiaHelper();
        vuforiaHelper.setupVuforia();
    }

    //todo write setup for this year's autonomous
    // note: not currently in use
    /*
     allows the driver to decide which autonomous routine should be run during the match through
     gamepad input
    */
    public void runSetUp()
    {


        telemetry.log().add("Setup finished.");
    }

    //todo needs to be changed for mecanum
    // use encoders to make the robot drive to a specified location
    public void driveToPosition(double targetX, double targetY) throws InterruptedException
    {

    }

    // tell the robot to turn to a specified angle
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

    // we use this function to determine the color of jewels and knock them
    public void knockJewel (boolean isBlueSide, boolean isLeftJewelBlue) throws InterruptedException
    {
        if(!isDriveTrainAttached)
        {
            return;
        }
        jewelJostlerServoToggler.toggle();
        pause(1000);

        if(isBlueSide)
        {
            if (isLeftJewelBlue)
            {
                turnTo(-90);
            }
            else
            {
                turnTo(90);
            }
        }
        else
        {
            if(isLeftJewelBlue)
            {
                turnTo(90);
            }
            else
            {
                turnTo(-90);
            }
        }

        jewelJostlerServoToggler.toggle();
        pause(1000);
    }

    //todo change to be based on encoder input
    // specialized method for driving the robot in autonomous
    public void moveRobot(double driveAngle, double drivePower, int pause) throws InterruptedException
    {
        driveMecanum(driveAngle, drivePower, 0.0);
        pauseWhileUpdating(pause);
        stopAllDriveMotors();
    }

    // gives the robot time to update state machines
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
