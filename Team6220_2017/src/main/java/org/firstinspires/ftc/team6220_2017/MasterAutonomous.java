package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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

    VuforiaHelper.BlueJewel blueJewel = VuforiaHelper.BlueJewel.UNDETERMINED;

    // Stores orientation of robot
    double currentAngle = 0.0;

    /*
    // Use for more advanced auto
    ArrayList<Alliance> routine = new ArrayList<>();
    RoutineOption routineOption = RoutineOption.;
    int delay = 0;
    */

    // todo Implement runSetup()
    // Used for object initializations only necessary in autonomous
    void initializeAuto() throws InterruptedException
    {
        // We don't want to run the arm during autonomous
        isArmAttached = false;

        initializeRobot();

        vuforiaHelper = new VuforiaHelper();
        vuforiaHelper.setupVuforia();
    }

    // Note: not currently in use
    /*
     Allows the 1st driver to decide which autonomous routine should be run during the match through
     gamepad input
    */
    void runSetup()
    {
        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();

        // Ensure log can't overflow
        telemetry.log().setCapacity(3);
        telemetry.log().add("Alliance Blue/Red = X/B");
        telemetry.log().add("Balancing stone Left/Right = Left/Right bumper");

        boolean settingUp = true;

        while (settingUp)
        {
            // Finds the time elapsed each loop
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();


            // Select alliance
            if (driver1.isButtonJustPressed(Button.X))
                isBlueSide = true;
            else if (driver1.isButtonJustPressed(Button.B))
                isBlueSide = false;

            // Select starting balancing stone
            else if (driver1.isButtonJustPressed(Button.LEFT_BUMPER))
                isLeftBalancingStone = true;
            else if (driver1.isButtonJustPressed(Button.RIGHT_BUMPER))
                isLeftBalancingStone = false;

            // If the driver presses start, we exit setup
            else if (driver1.isButtonJustPressed(Button.START))
                settingUp = false;

            // Display the current setup
            telemetry.addData("Is robot on blue alliance: ", isBlueSide);
            telemetry.addData("Is robot on left balancing stone: ", isLeftBalancingStone);


            updateCallback(eTime);
            telemetry.update();
            idle();
        }

        telemetry.log().add("Setup finished.");
    }

    // Tell the robot to turn to a specified angle
    public void turnTo(double targetAngle)
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
            RotationFilter.roll(angleDiff);
            turningPower = RotationFilter.getFilteredValue();

            // Make sure turningPower doesn't go above maximum power
            if (Math.abs(turningPower) > 1.0)
            {
                turningPower = Math.signum(turningPower);
            }

            // Makes sure turningPower doesn't go below minimum power
            if(Math.abs(turningPower) < Constants.MINIMUM_TURNING_POWER)
            {
                turningPower = Math.signum(turningPower) * Constants.MINIMUM_TURNING_POWER;
            }

            // Turns robot
            driveMecanum(0.0, 0.0, -turningPower);

            telemetry.addData("angleDiff: ", angleDiff);
            telemetry.addData("Turning Power: ", turningPower);
            telemetry.addData("Orientation: ", currentAngle);
            telemetry.update();
            idle();
        }

        stopDriveMotors();
    }

    // We use this method to score a jewel once its color has been determined
    public void knockJewel (VuforiaHelper.BlueJewel blueJewel, boolean isBlueSide) throws InterruptedException
    {
        verticalJewelServoToggler.toggle();
        pauseWhileUpdating(0.7);

        if(isBlueSide)
        {
            if (blueJewel == VuforiaHelper.BlueJewel.LEFT)
            {
                lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_RIGHT);
            }
            else if (blueJewel == VuforiaHelper.BlueJewel.RIGHT)
            {
                lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_LEFT);
            }
            // Do nothing if undetermined
        }
        else
        {
            if(blueJewel == VuforiaHelper.BlueJewel.LEFT)
            {
                lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_LEFT);
            }
            else if (blueJewel == VuforiaHelper.BlueJewel.RIGHT)
            {
                lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_RIGHT);
            }
            // Once again, do nothing if undetermined
        }
        pauseWhileUpdating(0.7);

        lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_NEUTRAL);
        verticalJewelServoToggler.toggle();
        pauseWhileUpdating(0.5);
    }

    // Specialized method for driving the robot in autonomous
    void moveRobot(double driveAngle, double drivePower, double pause) throws InterruptedException
    {
        driveMecanum(driveAngle, drivePower, 0.0);
        pauseWhileUpdating(pause);
        stopDriveMotors();
    }

    // todo Add absolute coordinates and code to prevent turning while using driveToPosition
    //todo sideways translation does not work
    // Uses encoders to make the robot drive to a specified relative position
    void driveToPosition(double deltaX, double deltaY, double maxPower) throws InterruptedException
    {
        // Find distance between robot and its destination
        double distanceToTarget = calculateDistance(deltaX, deltaY);
        // Variables set every loop-------------------
        double posFL;
        double posFR;
        double posBL;
        double posBR;
        double encX;
        double encY;
        double xDiff;
        double yDiff;
        double driveAngle;
        //---------------------------------------------

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Check to see if robot has arrived at destination within tolerances
        while (distanceToTarget > Constants.POSITION_TOLERANCE_MM && opModeIsActive())
        {
            // Store current encoder values as temporary variables
            posFL = motorFL.getCurrentPosition();
            posFR = motorFR.getCurrentPosition();
            posBL = motorBL.getCurrentPosition();
            posBR = motorBR.getCurrentPosition();

            // Average encoder differences to find translational x and y components (averaging values
            // gives more reliable results than using a single motor).  Motors turn differently
            // when translating, so signs on FR and BL must be flipped
            encX = (-posFL - posFR + posBL + posBR) / 4;
            encY = (-posFL + posFR - posBL + posBR) / 4;

            xDiff = deltaX - encX;
            yDiff = deltaY - encY;

            // Recalculate remaining distance each loop
            distanceToTarget = calculateDistance(xDiff, yDiff);

            // Power is proportional to distance from target
            double drivePower = maxPower * distanceToTarget;
            // Send drivePower through PID filter to adjust range and minimize oscillation
            TranslationFilter.roll(drivePower);
            double adjustedDrivePower = TranslationFilter.getFilteredValue();

            // Ensure robot doesn't approach target position too slowly
            if (Math.abs(adjustedDrivePower) < Constants.MINIMUM_DRIVE_POWER)
            {
                adjustedDrivePower = Math.signum(adjustedDrivePower) * Constants.MINIMUM_DRIVE_POWER;
            }

            // Deals with the fact that inverse tangent only returns an angle between -90 and 90
            // degrees.  We want to be able to drive at angles greater than 90 degrees, so the
            // range of atan2 must be modified
            if (deltaX >= 0)
                driveAngle = Math.toDegrees(Math.atan2(yDiff, xDiff));
            else
                driveAngle = Math.toDegrees(Math.atan2(yDiff, xDiff)) + 180;

            driveMecanum(driveAngle, adjustedDrivePower, 0.0);

            telemetry.addData("X remaining: ", xDiff);
            telemetry.addData("Y remaining: ", yDiff);
            telemetry.addData("Drive Angle: ", driveAngle);
            telemetry.addData("Power: ", adjustedDrivePower);
            telemetry.update();
            idle();
        }

        stopDriveMotors();
    }

    // todo Decide whether to keep or remove old method
    /*
    // todo Add absolute coordinates and code to prevent turning while using driveToPosition
    //todo sideways translation does not work
    // Uses encoders to make the robot drive to a specified relative position
    void driveToPosition(double deltaX, double deltaY, double maxPower) throws InterruptedException
    {
        // Find distance between robot and its destination
        double distanceToTarget = calculateDistance(deltaX, deltaY);

        // Store old values for drive encoders
        double lastEncoderFL = motorFL.getCurrentPosition();
        double lastEncoderFR = motorFR.getCurrentPosition();
        double lastEncoderBL = motorBL.getCurrentPosition();
        double lastEncoderBR = motorBR.getCurrentPosition();

        // Check to see if robot has arrived at destination within tolerances
        while (distanceToTarget > Constants.POSITION_TOLERANCE_MM && opModeIsActive())
        {
            // Store current encoder values as temporary variables
            double posFL = motorFL.getCurrentPosition();
            double posFR = motorFR.getCurrentPosition();
            double posBL = motorBL.getCurrentPosition();
            double posBR = motorBR.getCurrentPosition();
            // Changes in encoder values between loops
            double encDiffFL = posFL - lastEncoderFL;
            double encDiffFR = posFR - lastEncoderFR;
            double encDiffBL = posBL - lastEncoderBL;
            double encDiffBR = posBR - lastEncoderBR;

            // Save old encoder values for next loop
            lastEncoderFL = posFL;
            lastEncoderFR = posFR;
            lastEncoderBL = posBL;
            lastEncoderBR = posBR;

            // Average encoder differences to find translational x and y components.  Motors turn
            // differently when translating, so signs on FR and BL must be flipped
            double encDiffX = (-encDiffFL - encDiffFR + encDiffBL + encDiffBR) / 4;
            double encDiffY = (-encDiffFL + encDiffFR - encDiffBL + encDiffBR) / 4;
            //double encDiffY = -encDiffFL;
            //double encDiffX = -encDiffFR;

            // Translation distance is reduced by a factor of sqrt(2) due to mecanum wheels
            deltaX -= Constants.MM_PER_ANDYMARK_TICK * encDiffX / Math.sqrt(2);
            deltaY -= Constants.MM_PER_ANDYMARK_TICK * encDiffY;

            // Recalculate value each loop
            distanceToTarget = calculateDistance(deltaX, deltaY);

            double driveAngle;
            // Deals with the fact that inverse tangent only returns an angle between -90 and 90
            // degrees.  We want to be able to drive at angles greater than 90 degrees, so the
            // range of atan2 must be modified
            if (deltaX >= 0)
                driveAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
            else
                driveAngle = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 180;

            // Power is proportional to distance from target
            double drivePower = maxPower * distanceToTarget;
            // Send drivePower through PID filter to adjust range and minimize oscillation
            TranslationFilter.roll(drivePower);
            double adjustedDrivePower = TranslationFilter.getFilteredValue();

            // Ensure robot doesn't approach target position too slowly
            if (Math.abs(adjustedDrivePower) < Constants.MINIMUM_DRIVE_POWER)
            {
                adjustedDrivePower = Math.signum(adjustedDrivePower) * Constants.MINIMUM_DRIVE_POWER;
            }

            // todo DRIVE ANGLE HELD CONSTANT FOR TESTING
            driveMecanum(driveAngle, adjustedDrivePower, 0.0);

            telemetry.addData("X remaining: ", deltaX);
            telemetry.addData("Y remaining: ", deltaY);
            telemetry.addData("Drive Angle: ", driveAngle);
            telemetry.addData("Adjusted Drive Power: ", adjustedDrivePower);
            telemetry.update();
            idle();
        }

        stopDriveMotors();
    }
    */

    // todo Implement global coordinates (not a priority)
    // Updates robot's coordinates and angle
    public void updateLocation()
    {
        currentAngle = getAngularOrientationWithOffset();

        /*
        // Calculate how much drive motors have turned since last update
        int deltaFL = motorFL.getCurrentPosition() - lastEncoderFL;
        int deltaFR = motorFR.getCurrentPosition() - lastEncoderFR;
        int deltaBL = motorBL.getCurrentPosition() - lastEncoderBL;
        int deltaBR = motorBR.getCurrentPosition() - lastEncoderBR;

        // Average encoder ticks to find translational x and y components. deltaFR and deltaBL are
        // negative because they turn differently when translating
        double deltaX = (deltaFL - deltaFR - deltaBL + deltaBR) / 4;
        double deltaY = (deltaFL + deltaFR + deltaBL + deltaBR) / 4;

        // Convert to mm
        deltaX *= Constants.MM_PER_ANDYMARK_TICK;
        deltaY *= Constants.MM_PER_ANDYMARK_TICK;
        */

        /*
         Delta x and y are local values, so they need to be converted to global.
         Each local component has 2 global components, which are added to find the
         total global components of displacement. The global displacement components
         are then added to the previous position to set the new coordinates
        */
        /*
        robotX += deltaX * Math.sin(Math.toRadians(robotAngle)) + deltaY * Math.cos(Math.toRadians(robotAngle));
        robotY += deltaX * -Math.cos(Math.toRadians(robotAngle)) + deltaY * Math.sin(Math.toRadians(robotAngle));
        */

        telemetry.addData("currentAngle: ", currentAngle);
        telemetry.update();
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
