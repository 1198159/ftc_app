package org.firstinspires.ftc.team417;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

// there are no subclasses; this is the only one

@Autonomous(name="Master Autonomous", group = "Swerve")
// @Disabled

public class MasterAutonomous extends MasterOpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    final float mmPerInch = 25.4f;
    final float mmBotWidth = 18 * mmPerInch; // the robot width
    final float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    // declare "what team we're on" variables
    boolean isRedTeam; // are you on red team? (if not, you're blue)
    boolean isStartingPosOne;  // are you on starting position one? (if not, you're on position two
    double startDist; // the distance traveled depending on pos one or two
    int startDelay; // the time to delay the start if another team needs us to delay
    int delay = 0;
    int targetIndex; // specify what image target it is
    int targetDimX;  // specify x or y dim to use for alignment; red :x:0, blue :y:1
    int targetDimY;

    //double teamAngle;
    double pivotAngle; // will be 60, -60 depending on what team you're on
    double targetAngle; // the Vuforia angle to align to depending on what team you're on

    float[] targetPos = {1524, mmFTCFieldWidth}; // target position x and y with an origin right between the driver stations
    VuforiaNavigation VuforiaNav = new VuforiaNavigation();

    double Kmove = 1.0f/600.0f; // speed is proportional to error
    double Kpivot = 1.0f/150.0f;

    double TOL = 100;
    double TOL_ANGLE = 5;

    //CodeReview: add a comment before every method describing what the method does. It can be a single sentence.
    //CodeReview: If the method has input parameters, describe what they are.

    //CodeReview: MasterAutonomous shouldn't have a runOpMode because it's not meant to be an opmode by itself.
    //            Instead, you should create one or more subclasses of MasterAutonomous for each of your autonomous programs.
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        VuforiaNav.initVuforia();
        telemetry.addData("Path", "Select Team and Pos...");

        // Wait until we're told to go
        while (!isStarted())
        {
            // allow driver to choose a team
            if (gamepad1.b)
            {
                isRedTeam = true;
            }

            if (gamepad1.x)
            {
                isRedTeam = false;
            }

            // select position one or two, one is closer to the origin
            if (gamepad1.y)
            {
                isStartingPosOne = true;
            }
            if (gamepad1.a)
            {
                isStartingPosOne = false;
            }

            if (isRedTeam)
            {
                if (isStartingPosOne)
                {
                    telemetry.addData("Team: ", "Red 1");
                }
                else
                {
                    telemetry.addData("Team: ", "Red 2");
                }
            }
            else
            {
                if (isStartingPosOne)
                {
                    telemetry.addData("Team: ", "Blue 1");
                }
                else
                {
                    telemetry.addData("Team: ", "Blue 2");
                }
            }
            telemetry.update();
            idle();
        }
        telemetry.update();

        if (isRedTeam) // if team RED
        {
            if (isStartingPosOne)
            {
                // OPTION RED ONE (TOOLS)
                startDelay = 2000;
                pivotAngle = 55; // pivot this amount before acquiring target
                targetAngle = 0; // Vuforia angle
                startDist = 90;
                targetIndex = 1;
                targetPos[0] = 2743.2f;
                targetPos[1] = mmFTCFieldWidth;
                //telemetry.addData("Team: ", "Red 1"); // display what team we're on after choosing with the buttons
            }
            else
            {
                // OPTION RED TWO (GEARS)
                startDelay = 0;
                pivotAngle = 55; // pivot this amount before acquiring target
                targetAngle = 0; // Vuforia angle
                startDist = 50;
                targetIndex = 3;
                targetPos[0] = 1524;
                targetPos[1] = mmFTCFieldWidth;
                //telemetry.addData("Team: ", "Red 2"); // display what team we're on after choosing with the buttons
            }
            targetDimY = 0; // x
            targetDimY = 1;
            telemetry.update();
        }
        else // if team BLUE
        {
            if (isStartingPosOne)
            {
                // OPTION BLUE ONE (LEGOS)
                startDelay = 2000;
                pivotAngle = -55; // recalc pivot?? also for red one??
                targetAngle = -90;
                startDist = 90;
                targetIndex = 2;
                targetPos[0] = mmFTCFieldWidth;
                targetPos[1] = 2743.2f;
                //telemetry.addData("Team: ", "Blue 1");
            }
            else
            {
                // OPTION BLUE TWO (WHEELS)
                startDelay = 0;
                pivotAngle = -55;
                targetAngle = -90;
                startDist = 50;
                targetIndex = 0;
                targetPos[0] = mmFTCFieldWidth;
                targetPos[1] = 1524;
                //telemetry.addData("Team: ", "Blue 2");
            }
            targetDimX = 1; // y
            targetDimY = 0;
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        VuforiaNav.startTracking();
        //     pause(startDelay);
        VuforiaNav.getLocation();

        pause(delay);

        /*
        while (opModeIsActive()) {

            VuforiaNav.getLocation();
            telemetry.addData("tracking ", VuforiaNav.isVisible() ? "Visible" : "Not Visible");
            if (VuforiaNav.lastLocation != null)
            {
                telemetry.addData("location ", format(VuforiaNav.lastLocation));
            }

            telemetry.update();
            //pause(500);
        }
        */


// TESTS

        TOL = 30;
        TOL_ANGLE = 1;
        Kmove = 1.0/800.0;
        Kpivot = 1.0/150.0;

/*
        telemetry.addData("Path", "diagonal");
        telemetry.update();
        pivotMove(200, 200, 0, 0.8, 6);
        telemetry.addData("Path", "DONE");
        telemetry.update();
        pause(2000);

        telemetry.addData("Path", "right");
        telemetry.update();
        pivotMove(200, 0, 0, 0.8, 3);
        pause(3000);

        telemetry.addData("Path", "left");
        telemetry.update();
        pivotMove(-200, 0, 0, 0.8, 3);
        pause(3000);

        telemetry.addData("Path", "pivot left");
        telemetry.update();
        pivotMove(0, 0, 15, 0.8, 3);
        pause(3000);

        telemetry.addData("Path", "pivot right");
        telemetry.update();
        pivotMove(0, 0, -15, 0.8, 3);

        telemetry.addData("Path", "Done (end it!)");
        telemetry.update();
        pause(30000);

*/
        //alignPivotVuforia(targetAngle, 700, 4);

// START OF AUTONOMOUS

        telemetry.addData("Path", "start forwards");
        telemetry.update();
        // go towards target
        forwards(startDist, 0, 0.7, 3);  // inches, speed, timeout
        pause(100);

        telemetry.addData("Path", "pivot 60");
        telemetry.update();
        // pivot to face target
        pivot(pivotAngle, 0.7); // make sure IMU is on
        pause(100);

        telemetry.addData("Path", "align pivot vuf");
        telemetry.update();
        // align sideways with image
        alignPivotVuforia(0.7, 700, 3);
        pause(100);
        /*
        pivotVuforia(targetAngle, 0.5);
        pause(100);
        alignVuforia(0.6, 700, 3);   // speed, timeout
        pause(100);
        pivotVuforia(targetAngle, 0.5);
        */

        do
        {
            VuforiaNav.getLocation(); // update target location and angle
        }
        while (VuforiaNav.lastLocation == null);

// detect beacon color of left side: 0 is blue, 1 is red
        int beaconColor = VuforiaNav.GetBeaconColor();
        telemetry.log().add(String.format("LeftSide: %f, RightSide: %f", VuforiaNav.leftColorHSV[0], VuforiaNav.rightColorHSV[0]));
        telemetry.log().add(String.format("Returned Color: %d", beaconColor));
        if (isRedTeam)
        {
            telemetry.log().add(String.format("team red"));
        }
        else
        {
            telemetry.log().add(String.format("team blue"));
        }
        telemetry.update();

// shift left or right before pushing button
        if (beaconColor == 0)   // if left side beacon is blue
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                forwards(0, 2.5, 0.25, 3);   // shift right
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                forwards(0, -2.5, 0.25, 4);   // shift left
            }
        }
        else if (beaconColor == 1)  // if left side beacon is red
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                forwards(0, -2.5, 0.25, 4);   // shift left
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                forwards(0, 2.5, 0.25, 3);   // shift right
                pause(100);
            }
        }
        else // when the color is unknown
        {
            telemetry.addData("Path", "unknown color, going back");
            telemetry.update();
            forwards(-5, 0, 0.5, 3);
        }

        //CodeReview: do you still try to push the button if the color is unknown?
        //            Or is this wasted movement because you backed up a moment ago?

        telemetry.addData("Path", "pushing button");
        telemetry.update();
        forwards(17, 0, 0.25, 3); // push the button (first target)!!
        telemetry.log().add(String.format("pushed first button"));
        pause(100);

        // back up and align once again
        telemetry.addData("Path", "back up and align");
        telemetry.update();
        forwards(-20, 0, 0.3, 3);
        pivotVuforia(targetAngle, 0.3);

// CORNER VORTEX OPTION
        if (isRedTeam) pivot(-100, 0.6); // if red team, pivot left
        else pivot(100, 0.6);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        forwards(-30, 0, 0.5, 3);

        // lock motors

// CENTER VORTEX OPTION
/*        if (isRedTeam) pivot(50, 0.6);
        else pivot(-50, 0.6);
        forwards(-30, 0, 0.6, 3);
        //test
        //test2
        //test3
*/
/*

// determine next beacon target
        if (isRedTeam) // if team RED
        {
            // OPTION RED ONE (TOOLS)
            targetIndex = 1;
            targetPos[0] = 2743.2f;
            targetPos[1] = mmFTCFieldWidth;
            //telemetry.addData("Team: ", "Red 1"); // display what team we're on after choosing with the buttons
        }
        else // if team BLUE
        {
            // OPTION BLUE ONE (LEGOS)
            targetIndex = 2;
            targetPos[0] = mmFTCFieldWidth;
            targetPos[1] = 2743.2f;
            //telemetry.addData("Team: ", "Blue 1");
        }

// shift to new target!!
        telemetry.addData("Path", "shift to new target");
        telemetry.update();
        if (beaconColor == 0) // if left side blue
        {
            if (isRedTeam) // move shorter
            {
                forwards(0, 36, 0.6, 4);
            }
            else // move longer
            {
                forwards(0, -38, 0.6, 4);
            }
        }
        else if (beaconColor == 1) // if left side red
        {
            if (isRedTeam) // move longer
            {
                forwards(0, 38, 0.6, 4);
            }
            else // move shorter
            {
                forwards(0, -36, 0.6, 4);
            }
        }

        else // if color is unknown
        {
            if (isRedTeam) // move positive
            {
                forwards(0, 35, 0.6, 4);
            }
            else // move shorter
            {
                forwards(0, -35, 0.6, 4);
            }
        }
        pause(1000);

        //CodeReview: you didn't handle the case where the beaconColor was unknown.
        //            Don't you still want to move to the next beacon?

        telemetry.addData("Path", "back up");
        telemetry.update();
        forwards(-4, 0, 0.4, 2);

        VuforiaNav.lastLocation = null;

        do
        {
            VuforiaNav.getLocation(); // update target location and angle
        }
        while (VuforiaNav.lastLocation == null);

// align on new target
        telemetry.addData("Path", "align on new target");
        telemetry.update();
        alignPivotVuforia(0.7, 700, 3);
        */
        /*
        pivotVuforia(targetAngle, 0.5);
        alignVuforia(0.4, 700, 2);
        pivotVuforia(targetAngle, 0.5);
        */

        /*
        // detect beacon color of left side: 0 is blue, 1 is red
        beaconColor = VuforiaNav.GetBeaconColor();

        if (beaconColor == 0)   // if left side beacon is blue
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                forwards(0, 2.5, 0.25, 3);   // shift right
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                forwards(0, -2, 0.25, 4);   // shift left
            }
        }
        else if (beaconColor == 1)  // if left side beacon is red
        {
            if (isRedTeam)     // red team
            {
                telemetry.addData("Path", "shift left");
                telemetry.update();
                forwards(0, -2, 0.25, 4);   // shift left
            }
            else    // blue team
            {
                telemetry.addData("Path", "shift right");
                telemetry.update();
                forwards(0, 2.5, 0.25, 3);   // shift right
                pause(100);
            }
        }
        else if (beaconColor == 2) // used to be just else
        {
            forwards(-5, 0, 0.5, 3);
        }

        telemetry.addData("Path", "align angle");
        telemetry.update();
        pivotVuforia(targetAngle, 0.3);

        telemetry.addData("Path", "push button");
        telemetry.update();
        forwards(17, 0, 0.25, 3); // push the button
        pause(100);
        forwards(-10, 0, 0.5, 3);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        pause(10000);
*/
    }

    public void initializeRobot()
    {
        super.initializeHardware(); // call master op mode's init method

        // zero the motor controllers before running, don't know if motors start out at zero
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // run with encoder mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // treat back side (camera) as front
        // reverse left side motors instead of right side motors
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        motorFrontLeft.setMaxSpeed(2700);   // try this setting from 8923
        motorFrontRight.setMaxSpeed(2700);   // try this setting from 8923
        motorBackLeft.setMaxSpeed(2700);   // try this setting from 8923
        motorBackRight.setMaxSpeed(2700);   // try this setting from 8923

        //Set up telemetry data
        // We show the log in oldest-to-newest order, as that's better for poetry
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        telemetry.log().setCapacity(4);
        //configureDashboard();
    }

    // drive forwards/backwards/horizontal left and right function
    // forwardsInches: how many inches forward
    // horiInches: how many x axis inches you want to go
    // speed: the max speed
    public void forwards(double forwardInches, double horiInches, double speed, double timeout) throws InterruptedException
    {
        Kmove = 1.0/1200.0;
        TOL = 20;

        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;

        int errorFL;
        int errorFR;
        int errorBL;
        int errorBR;

        double speedFL;
        double speedFR;
        double speedBL;
        double speedBR;

        newTargetFL = motorFrontLeft.getCurrentPosition() + (int) Math.round(COUNTS_PER_INCH * forwardInches) + (int) Math.round(COUNTS_PER_INCH * horiInches * 1.414);
        newTargetFR = motorFrontRight.getCurrentPosition() + (int) Math.round(COUNTS_PER_INCH * forwardInches) - (int) Math.round(COUNTS_PER_INCH * horiInches * 1.414);
        newTargetBL = motorBackLeft.getCurrentPosition() + (int) Math.round(COUNTS_PER_INCH * forwardInches) - (int) Math.round(COUNTS_PER_INCH * horiInches * 1.414);
        newTargetBR = motorBackRight.getCurrentPosition() + (int) Math.round(COUNTS_PER_INCH * forwardInches) + (int) Math.round(COUNTS_PER_INCH * horiInches * 1.414);

        runtime.reset(); // used for timeout

        // wait until the motors reach the position
        do
        {
            errorFL = newTargetFL - motorFrontLeft.getCurrentPosition();
            speedFL = Math.abs(errorFL / Kmove);
            speedFL = Range.clip(speedFL, 0.2, speed);
            speedFL = speedFL * Math.signum(errorFL);

            errorFR = newTargetFR - motorFrontRight.getCurrentPosition();
            speedFR = Math.abs(errorFR / Kmove);
            speedFR = Range.clip(speedFR, 0.2, speed);
            speedFR = speedFR * Math.signum(errorFR);

            errorBL = newTargetBL - motorBackLeft.getCurrentPosition();
            speedBL = Math.abs(errorBL / Kmove);
            speedBL = Range.clip(speedBL, 0.2, speed);
            speedBL = speedBL * Math.signum(errorBL);

            errorBR = newTargetBR - motorBackRight.getCurrentPosition();
            speedBR = Math.abs(errorBR / Kmove);
            speedBR = Range.clip(speedBR, 0.2, speed);
            speedBR = speedBR * Math.signum(errorBR);

 /*
            if (Math.abs(errorFL) < TOL - 3)
            {
                speedFL = 0;
            }
            if (Math.abs(errorFR) < TOL - 3)
            {
                speedFR = 0;
            }
            if (Math.abs(errorBL) < TOL - 3)
            {
                speedBL = 0;
            }
            if (Math.abs(errorBR) < TOL - 3)
            {
                speedBR = 0;
            }
*/

            motorFrontLeft.setPower(speedFL);
            motorFrontRight.setPower(speedFR);
            motorBackLeft.setPower(speedBL);
            motorBackRight.setPower(speedBR);

            idle();
        }
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (Math.abs(errorFL) > TOL || Math.abs(errorFR) > TOL || Math.abs(errorBL) > TOL || Math.abs(errorBR) > TOL));

        // stop the motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    // aligns angularly with the image target
    public void pivotVuforia (double targetAngle, double speed)
    {
        double error;
        double curTurnAngle;
        double pivotSpeed;

        // run with encoder mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        do
        {
//            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
//            curTurnAngle = adjustAngles(angles.firstAngle) - startAngle;
            VuforiaNav.getLocation(); // update target location and angle
            //CodeReview: sometimes getLocation returns null. Sometimes Vuforia.lastLocation might be null. Does your code handle that case gracefully?

            do
            {
                VuforiaNav.getLocation(); // update target location and angle
                idle();
            }
            while (VuforiaNav.lastLocation == null);

            // now extract the angle out of "get location", andn stores your location
            curTurnAngle = Orientation.getOrientation(VuforiaNav.lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            curTurnAngle = adjustAngles(curTurnAngle);
            error =  targetAngle - curTurnAngle;
            pivotSpeed = speed * Math.abs(error) / 100;
            pivotSpeed = Range.clip(pivotSpeed, 0.2, 0.7); // limit abs speed
            pivotSpeed = pivotSpeed * Math.signum(error); // set the sign of speed


            pivot(error, 0.3);

            // allow some time for IMU to catch up

            /*
            if (Math.abs(error) < 2)

            {
                pause(15);
                // stop motors
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                pause(150);
            }
            */

            telemetry.log().add(String.format("CurAngle: %f, error: %f", curTurnAngle, error));
            idle();

        } while (opModeIsActive() && (Math.abs(error) > 2));    //&& Math.abs(errorP1) > 0.3 && Math.abs(errorP2) > 0.3) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }


    // move the robot hora. sideways to align with image target
    // look at double speed
    public void alignVuforiaTest (double speed, double distAway, double timeout)
    {
        float error;
        float xPos;
        float yPos;
        float errorX;
        float errorY;
        double robotErrorX;
        double robotErrorY;
        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;

        // add a new pos. Y in the future

        // run with encoder mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        do
        {
            //VuforiaNav.lastLocation = null;
            do
            {
                VuforiaNav.getLocation(); // update target location and angle
                idle();
            }
            while (VuforiaNav.lastLocation == null);  //CodeReview: this will hang your robot while Vuforia can't get its location. That could be a long time.

            xPos = VuforiaNav.lastLocation.getTranslation().getData()[0];
            yPos = VuforiaNav.lastLocation.getTranslation().getData()[1];
            errorX = targetPos[0] - xPos;
            errorY = targetPos[1] - yPos;

            // transform extrinsic to robot intrinsic
            robotErrorX = errorX * Math.cos(Math.toRadians(targetAngle)) + errorY * Math.sin(Math.toRadians(targetAngle));
            robotErrorY = -errorX * Math.sin(Math.toRadians(targetAngle)) + errorY * Math.cos(Math.toRadians(targetAngle));
            // shift position back 25 inches away from target image
            robotErrorY -= distAway;

            //error = targetPos[targetDimX] - xPos;

            // go sideways opposite of error
            newTargetFL = motorFrontLeft.getCurrentPosition() + (int) (COUNTS_PER_MM * (robotErrorX))
                    + (int) (COUNTS_PER_MM * (robotErrorY));
            newTargetFR = motorFrontRight.getCurrentPosition() + (int) (COUNTS_PER_MM * (-robotErrorX))
                    + (int) (COUNTS_PER_MM * (robotErrorY));
            newTargetBL = motorBackLeft.getCurrentPosition() + (int) (COUNTS_PER_MM * (-robotErrorX))
                    + (int) (COUNTS_PER_MM * (robotErrorY));
            newTargetBR = motorBackRight.getCurrentPosition() + (int) (COUNTS_PER_MM * (robotErrorX))
                    + (int) (COUNTS_PER_MM * (robotErrorY));

            motorFrontLeft.setTargetPosition(newTargetFL);
            motorFrontRight.setTargetPosition(newTargetFR);
            motorBackLeft.setTargetPosition(newTargetBL);
            motorBackRight.setTargetPosition(newTargetBR);

            motorFrontLeft.setPower(0.7);
            motorFrontRight.setPower(0.7);
            motorBackLeft.setPower(0.7);
            motorBackRight.setPower(0.7);
            runtime.reset();
            telemetry.log().add(String.format("X pos: %f, Y Pos: %f, NewXPos: %f", xPos, yPos, robotErrorX));
            //telemetry.update();

            // wait until the motors reach the position
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy()))
            {
                idle();
            };

            // stop the motors
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // error is in mm
        } while (opModeIsActive() && (Math.abs(robotErrorX) > 10.0));  //&& Math.abs(errorP1) > 0.3 && Math.abs(errorP2) > 0.3) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // move the robot hora. sideways to align with image target
    public void alignVuforia (double speed, double distAway, double timeout)
    {
        final float Kp = 1/600; // speed is proportional to error

        final float TOL = 10;

        float error;
        float xPos;
        float yPos;
        float errorX;
        float errorY;
        double robotErrorX;
        double robotErrorY;
        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;

        int errorFL;
        int errorFR;
        int errorBL;
        int errorBR;

        double speedFL;
        double speedFR;
        double speedBL;
        double speedBR;

        // run with encoder mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int loopCount = 0;
        do
        {
            VuforiaNav.lastLocation = null;
            do
            {
                VuforiaNav.getLocation(); // update target location and angle
                idle();
            }
            while (VuforiaNav.lastLocation == null);  //CodeReview: this will hang your robot while Vuforia can't get its location. That could be a long time.
            idle();

            xPos = VuforiaNav.lastLocation.getTranslation().getData()[0];
            yPos = VuforiaNav.lastLocation.getTranslation().getData()[1];
            errorX = targetPos[0] - xPos;
            errorY = targetPos[1] - yPos;

            // transform extrinsic to robot intrinsic
            robotErrorX = errorX * Math.cos(Math.toRadians(targetAngle)) + errorY * Math.sin(Math.toRadians(targetAngle));
            robotErrorY = -errorX * Math.sin(Math.toRadians(targetAngle)) + errorY * Math.cos(Math.toRadians(targetAngle));
            // shift position back 25 inches away from target image
            robotErrorY -= distAway;

            //error = targetPos[targetDimX] - xPos;

            // go sideways opposite of error
            newTargetFL = motorFrontLeft.getCurrentPosition() + (int) (COUNTS_PER_MM * (robotErrorX * 1.414))
                    + (int) (COUNTS_PER_MM * (robotErrorY));
            newTargetFR = motorFrontRight.getCurrentPosition() + (int) (COUNTS_PER_MM * (-robotErrorX * 1.414))
                    + (int) (COUNTS_PER_MM * (robotErrorY));
            newTargetBL = motorBackLeft.getCurrentPosition() + (int) (COUNTS_PER_MM * (-robotErrorX * 1.414))
                    + (int) (COUNTS_PER_MM * (robotErrorY));
            newTargetBR = motorBackRight.getCurrentPosition() + (int) (COUNTS_PER_MM * (robotErrorX * 1.414))
                    + (int) (COUNTS_PER_MM * (robotErrorY));

            motorFrontLeft.setTargetPosition(newTargetFL);
            motorFrontRight.setTargetPosition(newTargetFR);
            motorBackLeft.setTargetPosition(newTargetBL);
            motorBackRight.setTargetPosition(newTargetBR);

            // wait until the motors reach the position
            do
            {
                errorFL = newTargetFL - motorFrontLeft.getCurrentPosition();
                speedFL = Math.abs(errorFL * Kp); // make this a constant
                speedFL = Range.clip(speedFL, 0.2, speed);
                speedFL = speedFL * Math.signum(errorFL);

                errorFR = newTargetFR - motorFrontRight.getCurrentPosition();
                speedFR = Math.abs(errorFR * Kp);
                speedFR = Range.clip(speedFR, 0.2, speed);
                speedFR = speedFR * Math.signum(errorFR);

                errorBL = newTargetBL - motorBackLeft.getCurrentPosition();
                speedBL = Math.abs(errorBL * Kp);
                speedBL = Range.clip(speedBL, 0.2, speed);
                speedBL = speedBL * Math.signum(errorBL);

                errorBR = newTargetBR - motorBackRight.getCurrentPosition();
                speedBR = Math.abs(errorBR * Kp);
                speedBR = Range.clip(speedBR, 0.2, speed);
                speedBR = speedBR * Math.signum(errorBR);

                if (Math.abs(errorFL) < TOL)
                {
                    speedFL = 0;
                }
                if (Math.abs(errorFR) < TOL)
                {
                    speedFR = 0;
                }
                if (Math.abs(errorBL) < TOL)
                {
                    speedBL = 0;
                }
                if (Math.abs(errorBR) < TOL)
                {
                    speedBR = 0;
                }

                motorFrontLeft.setPower(speedFL);
                motorFrontRight.setPower(speedFR);
                motorBackLeft.setPower(speedBL);
                motorBackRight.setPower(speedBR);

                idle();
            }
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (Math.abs(errorFL) > TOL || Math.abs(errorFR) > TOL || Math.abs(errorBL) > TOL || Math.abs(errorBR) > TOL));

            runtime.reset();
            telemetry.log().add(String.format("X pos: %f, Y Pos: %f, NewXPos: %f, loop: %d", xPos, yPos, robotErrorX, loopCount));
            telemetry.update();
            loopCount++;

            // stop the motors
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // error is in mm
        } while (opModeIsActive() && (Math.abs(robotErrorX) > TOL));    //&& Math.abs(errorP1) > 0.3 && Math.abs(errorP2) > 0.3) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }


    // TODO: fix timeout
    // a combination of both the align and pivot function (WITHOUT VUFORIA)
    // angle has to be small otherwise won't work, this function moves and pivots robot
    public void pivotMove(double x, double y, double pivotAngle, double speed, double timeout)
    {
        // run with encoder mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Kmove = 1.0f/400.0f; // speed is proportional to error
        //Kpivot = 1.0f/100.0;

        //TOL = 100;
        //TOL_ANGLE = 5;

        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;

        int errorFL;
        int errorFR;
        int errorBL;
        int errorBR;

        double speedFL;
        double speedFR;
        double speedBL;
        double speedBR;

        double speedAbsFL;
        double speedAbsFR;
        double speedAbsBL;
        double speedAbsBR;

        double startAngle;
        double curTurnAngle;
        double pivotSpeed;
        double errorAngle;

        int pivotDst;
        final double ROBOT_DIAMETER_MM = 18.0 * 25.4;
        pivotDst = (int) ((pivotAngle / 360) * ROBOT_DIAMETER_MM * 3.1415 * COUNTS_PER_MM);

        newTargetFL = motorFrontLeft.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * (x * 1.414))
                + (int) Math.round(COUNTS_PER_MM * (y)) + pivotDst;
        newTargetFR = motorFrontRight.getCurrentPosition() - (int) Math.round(COUNTS_PER_MM * (x * 1.414))
                + (int) Math.round(COUNTS_PER_MM * (y)) - pivotDst;
        newTargetBL = motorBackLeft.getCurrentPosition() - (int) Math.round(COUNTS_PER_MM * (x * 1.414))
                + (int) Math.round(COUNTS_PER_MM * (y)) + pivotDst;
        newTargetBR = motorBackRight.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * (x * 1.414))
                + (int) Math.round(COUNTS_PER_MM * (y)) - pivotDst;

        runtime.reset(); // reset timer, which is used for loop timeout below

        // read starting angle
        startAngle = imu.getAngularOrientation().firstAngle;

        // wait until the motors reach the position
        // adjust robot angle during movement by adjusting speed of motors
        do
        {
            curTurnAngle = imu.getAngularOrientation().firstAngle - startAngle;
            curTurnAngle = adjustAngles(curTurnAngle);
            errorAngle =  pivotAngle - curTurnAngle;
            pivotSpeed = speed * errorAngle * Kpivot;
            pivotSpeed = Range.clip(pivotSpeed, -0.3, 0.3); // limit max pivot speed
            // pivotSpeed is added to each motor's movement speed

            errorFL = newTargetFL - motorFrontLeft.getCurrentPosition();
            speedFL = Kmove * errorFL;  // movement speed proportional to error
            speedFL += pivotSpeed;  // combine movement and pivot speeds
            speedAbsFL = Math.abs(speedFL);
            speedAbsFL = Range.clip(speedAbsFL, 0.2, speed);  // clip abs(speed)
            speedFL = speedAbsFL * Math.signum(speedFL);  // set sign of speed

            errorFR = newTargetFR - motorFrontRight.getCurrentPosition();
            speedFR = Kmove * errorFR;
            speedFR -= pivotSpeed;  // combine movement and pivot speeds
            speedAbsFR = Math.abs(speedFR);
            speedAbsFR = Range.clip(speedAbsFR, 0.2, speed);  // clip abs(speed)
            speedFR = speedAbsFR * Math.signum(speedFR);

            errorBL = newTargetBL - motorBackLeft.getCurrentPosition();
            speedBL = Kmove * errorBL;
            speedBL += pivotSpeed;  // combine movement and pivot speeds
            speedAbsBL = Math.abs(speedBL);
            speedAbsBL = Range.clip(speedAbsBL, 0.2, speed);  // clip abs(speed)
            speedBL = speedAbsBL * Math.signum(speedBL);

            errorBR = newTargetBR - motorBackRight.getCurrentPosition();
            speedBR = Kmove * errorBR;
            speedBR -= pivotSpeed;  // combine movement and pivot speeds
            speedAbsBR = Math.abs(speedBR);
            speedAbsBR = Range.clip(speedAbsBR, 0.2, speed);
            speedBR = speedAbsBR * Math.signum(speedBR);


            if (Math.abs(errorFL) < (TOL - 3) )
            {
                speedFL = 0;
            }
            if (Math.abs(errorFR) < (TOL - 3) )
            {
                speedFR = 0;
            }
            if (Math.abs(errorBL) < (TOL - 3) )
            {
                speedBL = 0;
            }
            if (Math.abs(errorBR) < (TOL - 3) )
            {
                speedBR = 0;
            }


            motorFrontLeft.setPower(speedFL);
            motorFrontRight.setPower(speedFR);
            motorBackLeft.setPower(speedBL);
            motorBackRight.setPower(speedBR);

            telemetry.log().add(String.format("spFL %f, spFR %f, spBL %f, spBR %f, time %f", speedFL, speedFR, speedBL, speedBR, runtime.seconds() ));
            telemetry.log().add(String.format("errFL %d, errFR %d, errBL %d, errBR %d", errorFL, errorFR, errorBL, errorBR));
            telemetry.update();
            idle();
            //if (runtime.seconds() > timeout) break;
        }
        while ( (opModeIsActive()) &&
                (runtime.seconds() < timeout) &&
                (
                        (Math.abs(errorFL) > TOL) || (Math.abs(errorFR) > TOL) || (Math.abs(errorBL) > TOL) || (Math.abs(errorBR) > TOL) ||
                                (Math.abs(errorAngle) > TOL_ANGLE)
                )
                );

        // stop the motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }


    // a combination of both the align and pivot function (WITH VUFORIA) using pivot move
    public void alignPivotVuforia (double speed, double distAway, double timeout)
    {
        final float VUFORIA_TOL = 10;
        final float VUFORIA_TOL_ANGLE = 1;

        float xPos;
        float yPos;
        float errorX;
        float errorY;
        double robotErrorX;
        double robotErrorY;

        double curRobotAngle;
        double pivotSpeed;
        double errorAngle;

        // run with encoder mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int loopCount = 0;
        do
        {
            VuforiaNav.lastLocation = null;
            do
            {
                VuforiaNav.getLocation(); // update target location and angle
                idle();
            }
            while (VuforiaNav.lastLocation == null);

            xPos = VuforiaNav.lastLocation.getTranslation().getData()[0];
            yPos = VuforiaNav.lastLocation.getTranslation().getData()[1];
            errorX = targetPos[0] - xPos;
            errorY = targetPos[1] - yPos;

            curRobotAngle = Orientation.getOrientation(VuforiaNav.lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            curRobotAngle = adjustAngles(curRobotAngle);
            errorAngle =  targetAngle - curRobotAngle;

            // transform extrinsic field coordinate to robot intrinsic coordinate so robot knows where to go
            robotErrorX = errorX * Math.cos(Math.toRadians(targetAngle)) + errorY * Math.sin(Math.toRadians(targetAngle));
            robotErrorY = -errorX * Math.sin(Math.toRadians(targetAngle)) + errorY * Math.cos(Math.toRadians(targetAngle));
            // shift position back 25 inches away from target image
            robotErrorY -= distAway;

// calls pivot move function here
            pivotMove(robotErrorX, robotErrorY, errorAngle, 0.5, 3); // 0.5 speed, 3 second timeout

            runtime.reset();
            telemetry.log().add(String.format("loop: %d", loopCount));
            telemetry.update();
            loopCount++;

            // stop the motors
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

            // error is in mm
        } while (opModeIsActive() && (Math.abs(robotErrorX) > VUFORIA_TOL && (Math.abs(robotErrorY) > VUFORIA_TOL)));    //&& Math.abs(errorP1) > 0.3 && Math.abs(errorP2) > 0.3) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }


    // drive at an angle function
    public void moveAngle(double forwardInches, double angle, double speed, double timeout) throws InterruptedException
    {
        double x;
        double y;

        double speedX;
        double speedY;

        double dstX;
        double dstY;

        // run to position mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        x = -Math.sin(Math.toRadians(angle));
        y =  Math.cos(Math.toRadians(angle));

        speedX = x * speed;
        speedY = y * speed;

        dstX = x * forwardInches;
        dstY = y * forwardInches;
        telemetry.log().add(String.format("Angle: %f, Dst: %f, X: %f, Y: %f", angle, forwardInches, dstX, dstY));

        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;

        newTargetFL = motorFrontLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * (dstX + dstY));
        newTargetFR = motorFrontRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * (-dstX + dstY));
        newTargetBL = motorBackLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * (-dstX + dstY));
        newTargetBR = motorBackRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * (dstX + dstY));

        motorFrontLeft.setTargetPosition(newTargetFL);
        motorFrontRight.setTargetPosition(newTargetFR);
        motorBackLeft.setTargetPosition(newTargetBL);
        motorBackRight.setTargetPosition(newTargetBR);

        motorFrontLeft.setPower(speedX + speedY);
        motorFrontRight.setPower(-speedX + speedY);
        motorBackLeft.setPower(-speedX + speedY);
        motorBackRight.setPower(speedX + speedY);
        runtime.reset();

        // wait until the motors reach the position
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy()))
        {
            idle();
        };

        // stop the motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // pivot using IMU
    public void pivot(double turnAngle, double speed)
    {
        double pivotSpeed;
        double startAngle;
        double curTurnAngle;

        // run with encoder mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //       angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        //       startAngle = adjustAngles(angles.firstAngle);
        startAngle = imu.getAngularOrientation().firstAngle;

        // read angle, record in starting angle variable
        // run motor
        // loop, current angle - start angle = error
        // if error is close to 0, stop motors

        double error = 100;
        double errorP1 = 100;
        double errorP2 = 100;

        do
        {
            errorP2 = errorP1;
            errorP1 = error;
//            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
//            curTurnAngle = adjustAngles(angles.firstAngle) - startAngle;
            curTurnAngle = imu.getAngularOrientation().firstAngle - startAngle;
            curTurnAngle = adjustAngles(curTurnAngle);
            error =  turnAngle - curTurnAngle;
            pivotSpeed = speed * Math.abs(error) / 100;
            pivotSpeed = Range.clip(pivotSpeed, 0.2, 0.7); // limit abs speed
            pivotSpeed = pivotSpeed * Math.signum(error); // set the sign of speed

            // positive angle means CCW rotation
            motorFrontLeft.setPower(pivotSpeed);
            motorFrontRight.setPower(-pivotSpeed);
            motorBackLeft.setPower(pivotSpeed);
            motorBackRight.setPower(-pivotSpeed);

            // allow some time for IMU to catch up
            if (Math.abs(error) < 2)
            {
                sleep(15);
                // stop motors
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                sleep(150);
            }

            telemetry.log().add(String.format("StartAngle: %f, CurAngle: %f, error: %f", startAngle, curTurnAngle, error));
            idle();

        } while (opModeIsActive() && (Math.abs(error) > 0.5 && Math.abs(errorP1) > 0.5 && Math.abs(errorP2) > 0.5) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }


    public void CornerVortexOption() throws InterruptedException
    {
        // CORNER VORTEX OPTION
        if (isRedTeam) pivot(-100, 0.6); // if red team, pivot left
        else pivot(100, 0.6);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        forwards(-30, 0, 0.5, 3);
    }


    public void CenterVOrtextOption() throws InterruptedException
    {
        // CENTER VORTEX OPTION
        if (isRedTeam) pivot(50, 0.6);
        else pivot(-50, 0.6);
        forwards(-30, 0, 0.6, 3);
    }

    // pushes the cap ball and parks on the center vortex
    public void PushCapBall() throws InterruptedException
    {
        forwards(45, 0, 0.7, 3);
        pause(500);
        forwards(5, 0, 0.7, 3);
    }


    // parks parks on corner vortex
    public void parkCornerVortex() throws InterruptedException
    {
        forwards(30, 0, 0.7, 3);
        if (isRedTeam)
        {
            forwards(0, -20, 0.8, 3); // horizontal left (20 inches)
        }
        else
        {
            forwards(0, 20, 0.8, 3); // horizontal right
        }
    }


    public void configureDashboard()
    {
        telemetry.addLine()
                .addData("Power | FrontLeft: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorFrontLeft.getPower());
                    }
                })
                .addData("Power | FrontRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorFrontRight.getPower());
                    }
                })
                .addData("Power | BackLeft: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBackLeft.getPower());
                    }
                })
                .addData("Power | BackRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBackRight.getPower());
                    }
                });
    }


    public String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();

    }
}


/* TABLE:
                 FL      FR      BL      BR
    rotate CCW   +        -      +        -
    forward      +        +      +        +
    right        +        -      -        +
    d. left      0        +      +        0
    d. right     +        0      0
    */