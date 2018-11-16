package org.firstinspires.ftc.team6220_2018;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;
import org.firstinspires.ftc.robotcore.external.Const;

import java.util.Locale;

/*
    Contains important methods for use in our autonomous programs
*/
abstract public class MasterAutonomous extends MasterOpMode
{
    OpenCVGold OpenCVVision;

    // Necessary for runSetup()
    DriverInput driverInput;

    // Initialize booleans and variables used in runSetup().  We do not need to specify our alliance
    // since the field is rotationally symmetric, halving our autonomous code.
     // Whether we start on the crater or depot side of the lander.
    public boolean isCraterStart = true;
     // Whether we park in our crater or our opponents'.
    public boolean isAllianceCraterFinal = true;
     // How long we want to wait before we start the match.
    public int matchDelay = 0;

    // Stores orientation of robot
    double currentAngle = 0.0;

    sampleFieldLocations goldLocation;

     /* Changeable values that tell us how the robot has to adjust during autonomous due to the
     gold mineral's three different locations.  For instance, if the gold mineral is in the right
     position, we must turn 30 degrees clockwise to view the mineral and later translate right to
     score it.  These movements must be compensated for later to make autonomous run consistently. */
    double mineralShift;
    double turnShift;


    enum sampleFieldLocations
    {
        left,
        center,
        right
    }


    /*
    // Use for more advanced auto
    ArrayList<Alliance> routine = new ArrayList<>();
    RoutineOption routineOption = RoutineOption.;
    */


    // Used for object initializations only necessary in autonomous
    void initializeAuto() throws InterruptedException
    {
        initializeRobot();

        motorHanger.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorHanger.setPower(0);

        OpenCVVision = new OpenCVGold();

        OpenCVVision.init(hardwareMap.appContext, ActivityViewDisplay.getInstance());
        OpenCVVision.setShowCountours(true);
        OpenCVVision.enable();
        //vuforiaHelper = new VuforiaHelper();
        //vuforiaHelper.setupVuforia();
    }


    // Allows the 1st driver to decide which autonomous routine should be run using gamepad input.
    void runSetup()
    {
        // Accounts for delay between initializing the program and starting TeleOp.
        lTime = timer.seconds();

        // Ensure log can't overflow
        telemetry.log().setCapacity(3);
        telemetry.log().add("Crater Start/Depot Start = Left Bumper/Right Bumper");
        telemetry.log().add("Final Destination - Alliance Crater/Opposing Crater = A/B");
        telemetry.log().add("Time Delay Increase/Decrease = D-Pad Up/D-Pad Down");

        boolean settingUp = true;

        while (settingUp)
        {
            // Finds the time elapsed each loop.
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            // Select start position.
            if (driver1.isButtonJustPressed(Button.LEFT_BUMPER))
                isCraterStart = true;
            else if (driver1.isButtonJustPressed(Button.RIGHT_BUMPER))
                isCraterStart = false;

            // Select parking location at end of match.
            if(driver1.isButtonJustPressed(Button.A))
                isAllianceCraterFinal = true;
            else if (driver1.isButtonJustPressed((Button.B)))
                isAllianceCraterFinal = false;

            // Select alliance.  We restrict our time delay from 0 to 10 seconds.
            if (driver1.isButtonJustPressed(Button.DPAD_UP) && matchDelay < 10)
                matchDelay++;
            else if (driver1.isButtonJustPressed(Button.DPAD_DOWN) && matchDelay > 0)
                matchDelay--;

            // If the driver presses start, we exit setup.
            else if (driver1.isButtonJustPressed(Button.START))
                settingUp = false;

            // Display the current setup
            telemetry.addData("Is robot starting by crater: ", isCraterStart);
            telemetry.addData("Is robot final destination alliance crater: ", isAllianceCraterFinal);
            telemetry.addData("Time delay", matchDelay);

            updateCallback(eTime);
            telemetry.update();
            idle();
        }

        telemetry.log().clear();
        telemetry.log().add("Setup finished.");
    }

    // Tell the robot to turn to a specified angle.  We can also limit the motor power while turning.
    public void turnTo(double targetAngle, double maxPower)
    {
        double turningPower;
        currentAngle = getAngularOrientationWithOffset();
        double angleDiff = normalizeRotationTarget(targetAngle, currentAngle);

        // Robot only stops turning when it is within angle tolerance
        while(Math.abs(angleDiff) >= Constants.ANGLE_TOLERANCE_DEG && opModeIsActive())
        {
            currentAngle = getAngularOrientationWithOffset();

            // Give robot raw value for turning power
            angleDiff = normalizeRotationTarget(targetAngle, currentAngle);

            // Send raw turning power through PID filter to adjust range and minimize oscillation
            rotationFilter.roll(angleDiff);
            turningPower = rotationFilter.getFilteredValue();

            // Make sure turningPower doesn't go above maximum power
            if (Math.abs(turningPower) > maxPower)
            {
                turningPower = maxPower * Math.signum(turningPower);
            }

            // Makes sure turningPower doesn't go below minimum power
            if(Math.abs(turningPower) < Constants.MINIMUM_TURNING_POWER)
            {
                turningPower = Math.signum(turningPower) * Constants.MINIMUM_TURNING_POWER;
            }

            // Turns robot
            driveMecanum(0.0, 0.0, turningPower);

            /*telemetry.addData("angleDiff: ", angleDiff);
            telemetry.addData("Turning Power: ", turningPower);
            telemetry.addData("Orientation: ", currentAngle);
            telemetry.update();*/
            idle();
        }

        stopDriveMotors();
    }


    // Specialized method for driving the robot in autonomous
    void moveRobot(double driveAngle, double drivePower, double pause) throws InterruptedException
    {
        driveMecanum(driveAngle, drivePower, 0.0);
        pauseWhileUpdating(pause);
        stopDriveMotors();
    }


    // todo Add global coordinates (not a priority)
    // Uses encoders to make the robot drive to a specified relative position.  Also makes use of the
    // imu to keep the robot at a constant heading during navigation.
    // **Note:  initDeltaX/Y are in mm.
    void driveToPosition(double initDeltaX, double initDeltaY, double maxPower) throws InterruptedException
    {
        // Variables set every loop-------------------
        double deltaX = initDeltaX;
        double deltaY = initDeltaY;
        double headingDiff = 0;

        double driveAngle;
        double drivePower;
        double adjustedDrivePower;
        double rotationPower;

         // Find distance between robot and its destination
        double distanceToTarget = calculateDistance(deltaX, deltaY);
        //---------------------------------------------
        double initHeading = getAngularOrientationWithOffset();

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Check to see if robot has arrived at destination within tolerances
        while (((distanceToTarget > Constants.POSITION_TOLERANCE_MM) || (headingDiff > Constants.ANGLE_TOLERANCE_DEG))&& opModeIsActive())
        {
            deltaX = initDeltaX - Constants.MM_PER_ANDYMARK_TICK * (-motorFL.getCurrentPosition() +
                    motorBL.getCurrentPosition() - motorFR.getCurrentPosition() + motorBR.getCurrentPosition()) / (4 * Math.sqrt(2));
            deltaY = initDeltaY - Constants.MM_PER_ANDYMARK_TICK * (-motorFL.getCurrentPosition() -
                    motorBL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBR.getCurrentPosition()) / 4;

            // Calculate how far off robot is from its initial heading
            headingDiff = normalizeRotationTarget(getAngularOrientationWithOffset(), initHeading);

            // Recalculate drive angle and distance remaining every loop
            distanceToTarget = calculateDistance(deltaX, deltaY);
            driveAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

            // Transform position and heading diffs to linear and rotation powers using filters----
            translationFilter.roll(distanceToTarget);
            drivePower = translationFilter.getFilteredValue();

            // Ensure robot doesn't approach target position too slowly
            if (Math.abs(drivePower) < Constants.MINIMUM_DRIVE_POWER)
            {
                drivePower = Math.signum(drivePower) * Constants.MINIMUM_DRIVE_POWER;
            }
            // Ensure robot doesn't ever drive faster than we want it to
            else if (Math.abs(drivePower) > maxPower)
            {
                drivePower = Math.signum(drivePower) * maxPower;
            }

             // Additional factor is necessary to ensure turning power is large enough
            rotationFilter.roll(-1.5 * headingDiff);
            rotationPower = rotationFilter.getFilteredValue();
            //-------------------------------------------------------------------------------------

            driveMecanum(driveAngle, drivePower, rotationPower);

            /*
            telemetry.addData("Encoder Diff x: ", deltaX);
            telemetry.addData("Encoder Diff y: ", deltaY);
            telemetry.addData("Drive Power: ", drivePower);
            telemetry.addData("Rotation Power: ", rotationPower);
            telemetry.update();
            */
            idle();
        }
        stopDriveMotors();
    }


    // Uses OpenCV to identify the location of the gold mineral.
    public void identifyGold ()
    {
        turnTo(-16.5,1.0);
        pauseWhileUpdating(1.0);
        // Gold is towards left of phone screen in horizontal  (rotated counter clockwise 90 degrees looking at it from the front).
        if (((OpenCVVision.getGoldRect().y + (OpenCVVision.getGoldRect().height / 2)) < Constants.GOLD_DIVIDING_LINE) && (OpenCVVision.getGoldRect().y > Constants.OPENCV_TOLERANCE_PIX))
        {
            goldLocation = sampleFieldLocations.center;
            telemetry.addLine("Center");
        }
        // Gold is towards right of phone screen in horizontal position (rotated counter clockwise 90 degrees looking at it from the front).
        else if (((OpenCVVision.getGoldRect().y + (OpenCVVision.getGoldRect().height / 2))) > Constants.GOLD_DIVIDING_LINE)
        {
            goldLocation = sampleFieldLocations.right;
            telemetry.addLine("Right");
        }
        // Gold is towards middle of phone screen in horizontal position (rotated counter clockwise 90 degrees looking at it from the front).
        else
        {
            goldLocation = sampleFieldLocations.left;
            telemetry.addLine("Left (default)");
        }
        turnTo(0,1.0);
        telemetry.addData("Gold",
                String.format(Locale.getDefault(), "(%d, %d)", (OpenCVVision.getGoldRect().x + (OpenCVVision.getGoldRect().width) / 2), (OpenCVVision.getGoldRect().y + (OpenCVVision.getGoldRect().height / 2))));
        telemetry.addData("currentAngle", currentAngle);
        telemetry.update();
    }


    // Use this method to knock the gold mineral given its location.
    public void knockGold (sampleFieldLocations goldLocation) throws InterruptedException
    {
        if (goldLocation == sampleFieldLocations.center)
        {
            mineralShift = 0;
            driveToPosition(mineralShift, Constants.MINERAL_FORWARD, 1.0);
            turnShift = 0;
        }
        else if (goldLocation == sampleFieldLocations.right)
        {
            mineralShift = Constants.MINERAL_SHIFT;
            driveToPosition(mineralShift, Constants.MINERAL_FORWARD, 1.0);
            turnShift = 30;
        }
        else
        {
            mineralShift = -Constants.MINERAL_SHIFT;
            driveToPosition(mineralShift, Constants.MINERAL_FORWARD, 1.0);
            turnShift = -30;
        }
    }
}
