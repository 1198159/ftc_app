package org.firstinspires.ftc.team6220_2017;

import android.graphics.Color;

import java.util.ArrayList;

/*
    Contains important methods for use in our autonomous programs
*/
abstract public class MasterAutonomous extends MasterOpMode
{
    // Necessary for runSetup()
    DriverInput driverInput;

    // Initialize booleans/variables used in runSetup()
    public boolean isBlueSide = true;
    public boolean isLeftBalancingStone = true;

    /*
    // Use for more advanced auto
    ArrayList<Alliance> routine = new ArrayList<>();
    RoutineOption routineOption = RoutineOption.;
    int delay = 0;
    */

    // todo Implement runSetup()
    // Used for object initializations only necessary in autonomous
    public void initializeAuto()
    {
        // We don't want to run the arm during autonomous
        isArmAttached = false;

        initializeHardware();

        vuforiaHelper = new VuforiaHelper();
        vuforiaHelper.setupVuforia();
    }

    // Note: not currently in use
    /*
     Allows the 1st driver to decide which autonomous routine should be run during the match through
     gamepad input
    */
    public void runSetup()
    {
        telemetry.log().add("Alliance Blue/Red = X/B");
        telemetry.log().add("Balancing stone Left/Right = Left/Right bumper");

        boolean settingUp = true;

        while (settingUp)
        {
            // Select alliance
            if (driver1.isButtonPressed(Button.X))
                isBlueSide = true;
            else if (driver1.isButtonPressed(Button.B))
                isBlueSide = false;

            // Select starting balancing stone
            else if (driver1.isButtonPressed(Button.LEFT_BUMPER))
                isLeftBalancingStone = true;
            else if (driver1.isButtonPressed(Button.RIGHT_BUMPER))
                isLeftBalancingStone = false;

            // If the driver presses start, we exit setup
            else if (gamepad1.start)
                settingUp = false;

            /*
             Wait for all buttons to be released before continuing. Otherwise, buttons are read
             continuously in setup and may cause issues (e.g., adding multiples of the same objective)
            */
            while(!driverInput.areAllButtonsReleased(gamepad1)) { idle(); }

            // Display the current routine
            telemetry.addData("Are we blue: ", isBlueSide);
            telemetry.addData("Are we on left balancing stone: ", isLeftBalancingStone);

            telemetry.update();
            idle();
        }

        telemetry.log().add("Setup finished.");
    }

    // todo needs to be changed for mecanum
    // use encoders to make the robot drive to a specified location
    public void driveToPosition(double targetX, double targetY) throws InterruptedException
    {

    }

    // Tell the robot to turn to a specified angle
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

    // We use this function to determine the color of jewels and score them
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

    // todo Change to be based on encoder input
    // Specialized method for driving the robot in autonomous
    public void moveRobot(double driveAngle, double drivePower, int pause) throws InterruptedException
    {
        driveMecanum(driveAngle, drivePower, 0.0);
        pauseWhileUpdating(pause);
        stopAllDriveMotors();
    }

    // Gives the robot time to update state machines
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
