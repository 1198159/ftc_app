package org.firstinspires.ftc.team6220_2017;

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

    // todo Needs to be changed for mecanum drive
    // Use encoders to make the robot drive to a specified location
    public void driveToPosition(double targetX, double targetY) throws InterruptedException
    {

    }

    // Tell the robot to turn to a specified angle
    public void turnTo(double targetAngle)
    {
        double turningPower;
        double currentAngle = getAngularOrientationWithOffset();
        double angleDiff = normalizeRotationTarget(targetAngle, currentAngle);

        // Robot only stops turning when it is within angle tolerance
        while(Math.abs(angleDiff) >= Constants.ANGLE_TOLERANCE && opModeIsActive())
        {
            currentAngle = getAngularOrientationWithOffset();

            // Gives robot its adjusted turning power
            angleDiff = normalizeRotationTarget(targetAngle, currentAngle);
            turningPower = Constants.TURNING_POWER_FACTOR * angleDiff;

            // Sends turningPower through PID filter to prevent oscillation
            RotationControlFilter.roll(turningPower);
            turningPower = RotationControlFilter.getFilteredValue();

            // Makes sure turn power doesn't go above maximum power
            if (Math.abs(turningPower) > 1.0)
            {
                turningPower = Math.signum(turningPower);
            }

            // Makes sure turn power doesn't go below minimum power
            if(Math.abs(turningPower) < Constants.MINIMUM_TURNING_POWER)
            {
                turningPower = Math.signum(Constants.MINIMUM_TURNING_POWER) * Constants.MINIMUM_TURNING_POWER;
            }

            // Turns robot
            driveMecanum(0.0, 0.0, -turningPower);

            telemetry.addData("angleDiff: ", angleDiff);
            telemetry.update();
            idle();
        }

        stopAllDriveMotors();
    }

    // We use this method to score a jewel once its color has been determined
    public void knockJewel (boolean isLeftBlue, boolean isBlueSide) throws InterruptedException
    {
        verticalJewelServoToggler.toggle();
        pause(500);

        if(isBlueSide)
        {
            if(isLeftBlue)
            {
                lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_RIGHT);
                //driveMecanum(180, 1.0, 0.0);
                stopAllDriveMotors();
            }
            else
            {
                lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_LEFT);
            }
        }
        else
        {
            if(isLeftBlue)
            {
                lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_LEFT);
            }
            else
            {
                lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_RIGHT);
            }
        }
        pause(500);

        lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_NEUTRAL);
        verticalJewelServoToggler.toggle();
    }

    // todo Change to be based on encoder input
    // Specialized method for driving the robot in autonomous
    public void moveRobot(double driveAngle, double drivePower, double pause) throws InterruptedException
    {
        driveMecanum(driveAngle, drivePower, 0.0);
        pauseWhileUpdating(pause);
        stopAllDriveMotors();
    }

    // Note:  time parameter is in seconds
    // Gives the robot time to update state machines
    void pauseWhileUpdating(double time)
    {
        lTime = timer.seconds();

        while(opModeIsActive() && (time > 0))
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();
            time -= eTime;
            telemetry.addData("eTime:", eTime);
            telemetry.addData("Seconds Remaining:", time);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }
}
