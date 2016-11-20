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


/**
 * Program used to control Drive-A-Bots.
 * This can be a good reference for drive controls.
 */
//CodeReview: you shouldn't register the master autonomous. You should create one or more subclasses of MasterAutonomous and register those.
//            That's useful in case you are experimenting with different autonomous programs: they can all use the methods you have in here.
@Autonomous(name="Master Autonomous", group = "Swerve")
// @Disabled

public class MasterAutonomous extends MasterOpMode
{
    // HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    //CodeReview: runtime is also declared in MasterOpMode. You don't need to declare this here. It's duplicating/overriding that variable.
    private ElapsedTime runtime = new ElapsedTime();

    //CodeReview: the following three variables have values that never change (in other words, they are constants).
    //            Mark them as such by using the keyword "final".
    //            This prevents someone from accidentally modifying the values later and makes it clearer that these are constants.
    //            I've fixed the first one as an example.
    final float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    // declare "what team we're on" variables
    boolean isRedTeam; //CodeReview: add comments describing what each variables is for. Some of the following variables don't have explanations
    boolean isPosOne;  //CodeReview: the name of this variable could be clearer. E.g., isStartingPositionOne
    double startDist; // the distance traveled depending on pos one or two
    int startDelay;
    int targetIndex; // specify what image target it is
    int targetDimX;      // specify x or y dim to use for alignment; red :x:0, blue :y:1
    int targetDimY;

    //double teamAngle; // will be 60, -60
    double pivotAngle;
    double targetAngle;

    float[] targetPos = {1524, mmFTCFieldWidth};
    VuforiaNavigation VuforiaNav = new VuforiaNavigation();


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
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

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
                isPosOne = true;
            }
            if (gamepad1.a)
            {
                isPosOne = false;
            }

            //CodeReview: it would be a good idea to have some indication that these choices were accepted (team, start position),
            //            e.g. a telemetry message or an LED on the robot showing the current state.


            //CodeReview: move the following code outside of this while loop.
            //            This code is executing every loop even if your choice hasn't changed.
            //            The only thing you really need here is the telemetry, and you only need to output that when
            //            the driver makes a choice (so you can do it in the keyhandling above).
            //            The other operations are just wasting cycles.

            if (isRedTeam) // if team RED
            {
                if (isPosOne)
                {
                    // OPTION RED ONE (TOOLS)
                    startDelay = 2000;
                    pivotAngle = 60; // pivot this amount before acquiring target
                    targetAngle = 0; // Vuforia angle
                    startDist = 80;
                    targetIndex = 1;
                    targetPos[0] = 2743.2f;
                    targetPos[1] = mmFTCFieldWidth;
                    telemetry.addData("Team: ", "Red 1"); // display what team we're on after choosing with the buttons
                }
                else
                {
                    // OPTION RED TWO (GEARS)
                    startDelay = 0;
                    pivotAngle = 60; // pivot this amount before acquiring target
                    targetAngle = 0; // Vuforia angle
                    startDist = 50;
                    targetIndex = 3;
                    targetPos[0] = 1524;
                    targetPos[1] = mmFTCFieldWidth;
                    telemetry.addData("Team: ", "Red 2"); // display what team we're on after choosing with the buttons
                }
                targetDimY = 0; // x
                targetDimY = 1;

            }
            else // if team BLUE
            {
                if (isPosOne)
                {
                    // OPTION BLUE ONE (LEGOS)
                    startDelay = 2000;
                    pivotAngle = -60; // recalc pivot?? also for red one??
                    targetAngle = -90;
                    startDist = 80;
                    targetIndex = 2;
                    targetPos[0] = mmFTCFieldWidth;
                    targetPos[1] = 2743.2f;
                    telemetry.addData("Team: ", "Blue 1");
                }
                else
                {
                    // OPTION BLUE TWO (WHEELS)
                    startDelay = 0;
                    pivotAngle = -60;
                    targetAngle = -90;
                    startDist = 50;
                    targetIndex = 0;
                    targetPos[0] = mmFTCFieldWidth;
                    targetPos[1] = 1524;
                    telemetry.addData("Team: ", "Blue 2");
                }
                targetDimX = 1; // y
                targetDimY = 0;
            }

            telemetry.update();
            idle();
        }
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        VuforiaNav.startTracking();
   //     sleep(startDelay);
        VuforiaNav.getLocation();

        /*
        while (opModeIsActive()) {

            VuforiaNav.getLocation();
            telemetry.addData("tracking ", VuforiaNav.isVisible() ? "Visible" : "Not Visible");
            if (VuforiaNav.lastLocation != null)
            {
                telemetry.addData("location ", format(VuforiaNav.lastLocation));
            }

            telemetry.update();
            //sleep(500);
        }
        */



        // START OF AUTONOMOUS
// TODO: test new functions!!

        /* HERE!
        // go towards target
        forwards(startDist, 0.5, 3);  // inches, speed, timeout
        sleep(500);

        // pivot to face target
        pivot(pivotAngle, 0.7);
        sleep(500);

        // align sideways with image
        pivotVuforia(targetAngle, 0.5);
        sleep(500);
        alignVuforia(0.5, 800, 3);   // speed, timeout
        sleep(500);
        pivotVuforia(targetAngle, 0.5);
        sleep(500);
        alignVuforia(0.5, 700, 3);   // speed, timeout
        sleep(500);
*/


        if (isPosOne)
        {
            PushCapBall();
        }
        else
        {
            parkCornerVortex();
        }




        /*

        do
        {
            VuforiaNav.getLocation(); // update target location and angle
        }
        while (VuforiaNav.lastLocation == null);


        // detect beacon color of left side: 0 - blue, 1 - red
        // TODO: handle failure of getBeaconColor
        int beaconColor = VuforiaNav.GetBeaconColor();


        // TODO:  Decide what to do after detecting beacon color
        //
        // if left blue and blue team, then slide left else slide right
        //   move forwards to press button

        // shift left or right before pushing button
        if (beaconColor == 0)   // blue
        {
            if (isRedTeam == false)     // blue team
            {
                //moveAngle(10, 90, 0.5, 3);   // no shift (left side)
            }
            else    // red team
            {
                moveAngle(5.25, -90, 0.5, 3);   // shift right
            }
        }
        else if (beaconColor == 1)  // if beacon is red
        {
            if (isRedTeam == false)     // blue team
            {
                moveAngle(5.25, -90, 0.5, 3);   // shift right
            }
            else    // red team
            {
                //moveAngle(10, 90, 0.5, 3);   // no shift (left side)
            }
        }
        forwards(23, 0.3, 3); // push the button
        sleep(200);
        forwards(-5, 0.3, 3);
        sleep(200);
        forwards(6.5, 0.3, 3);
        sleep(200);
        forwards(-10, 0.3, 3);
*/


        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // encoderDrive(DRIVE_SPEED, 30, 30, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        // encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        // encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(5000);
    }

    public void initializeRobot()
    {
        super.initializeHardware(); // call master op mode's init method

        // run to position mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        motorLift.setPower(0); //CodeReview: this is duplicated in MasterOpMode's initializeHardware method; you don't need it here.

        //Set up telemetry data
        // We show the log in oldest-to-newest order, as that's better for poetry
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        telemetry.log().setCapacity(3);
        //configureDashboard();
    }

    // drive forwards/backwards function
    public void forwards(double forwardInches, double speed, double timeout) throws InterruptedException
    {
        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;


        //CodeReview: when multiplying (COUNTS_PER_INCH * forwardInches), you are casting it to an (int) which will truncate its fractional value.
        //            There is a potential slight accuracy improvement by rounding this number instead of truncating it.
        //            The way to do that is to use (int) Math.round(COUNTS_PER_INCH * forwardInches);
        //            Note that the (int) cast is still needed because Math.round returns a long, but you're safe to do this because
        //            you won't be using bigger numbers than an int can hold.
        newTargetFL = motorFrontLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetFR = motorFrontRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetBL = motorBackLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetBR = motorBackRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);

        motorFrontLeft.setTargetPosition(newTargetFL);
        motorFrontRight.setTargetPosition(newTargetFR);
        motorBackLeft.setTargetPosition(newTargetBL);
        motorBackRight.setTargetPosition(newTargetBR);

        runtime.reset(); //CodeReview: add a comment to this line because it's not self-evident what this call is doing for you.
        motorFrontLeft.setPower(Math.abs(speed));
        motorFrontRight.setPower(Math.abs(speed));
        motorBackLeft.setPower(Math.abs(speed));
        motorBackRight.setPower(Math.abs(speed));

        // wait until the motors reach the position
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy()));

        // stop the motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

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
            }
            while (VuforiaNav.lastLocation == null);

            // now extract the angle out of "get location", andn stores your location
            curTurnAngle = Orientation.getOrientation(VuforiaNav.lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            curTurnAngle = adjustAngles(curTurnAngle);
            error =  targetAngle - curTurnAngle;
            pivotSpeed = speed * Math.abs(error) / 100;
            pivotSpeed = Range.clip(pivotSpeed, 0.18, 0.7); // limit abs speed
            pivotSpeed = pivotSpeed * Math.signum(error); // set the sign of speed


            pivot(error, 0.3);

            // allow some time for IMU to catch up

            /*
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
            */

            telemetry.log().add(String.format("CurAngle: %f, error: %f", curTurnAngle, error));

        } while (opModeIsActive() && (Math.abs(error) > 0.3));    //&& Math.abs(errorP1) > 0.3 && Math.abs(errorP2) > 0.3) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void centerImage (double speed, float timeout) // pivot in little steps and check after each pivot to see if error = 0
    {
        double error;
        double incDst;
        int newTargetFL;
        int newTargetFR;
        int newTargetBL;
        int newTargetBR;

        float x = VuforiaNav.GetImageCenter(targetIndex);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        do
        {
            x = VuforiaNav.GetImageCenter(targetIndex);
            error = 360 - x;      // center of camera frame 1280x720 resolution
            incDst = Math.signum(error) * 2;    // pivot dist
            // ccw rotation for pos pivot angle
            newTargetFL = motorFrontLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * incDst);
            newTargetFR = motorFrontRight.getCurrentPosition() - (int) (COUNTS_PER_INCH * incDst);
            newTargetBL = motorBackLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * incDst);
            newTargetBR = motorBackRight.getCurrentPosition() - (int) (COUNTS_PER_INCH * incDst);

            motorFrontLeft.setTargetPosition(newTargetFL);
            motorFrontRight.setTargetPosition(newTargetFR);
            motorBackLeft.setTargetPosition(newTargetBL);
            motorBackRight.setTargetPosition(newTargetBR);

            runtime.reset();
            motorFrontLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorBackLeft.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));

        }while (opModeIsActive() &&  // if there is time left and error less than 100 pixels
            (runtime.seconds() < timeout) &&
            (Math.abs(error) < 100));
    }

    // move the robot hora. sideways to align with image target
    public void alignVuforia (double speed, double distAway, double timeout)
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
            do
            {
                VuforiaNav.getLocation(); // update target location and angle
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
                    (motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy()));

            // stop the motors
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);

        } while (opModeIsActive() && (Math.abs(robotErrorX) > 3.0));    //&& Math.abs(errorP1) > 0.3 && Math.abs(errorP2) > 0.3) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    // drive at an angle function
    public void moveAngle(double forwardInches, double angle, double speed, double timeout) throws InterruptedException
    {
        double speedFL;
        double speedBL;
        double speedFR;
        double speedBR;

        double startAngle;
        double curTurnAngle;

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

        speedX = x*speed;
        speedY = y*speed;

        dstX = x*forwardInches;
        dstY = y*forwardInches;
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
        motorFrontRight.setPower(-speedX + speedY );
        motorBackLeft.setPower(-speedX + speedY );
        motorBackRight.setPower(speedX + speedY );
        runtime.reset();

        // wait until the motors reach the position
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy()));

        // stop the motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
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
        // if error is close to 0, then stop motors


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
            pivotSpeed = Range.clip(pivotSpeed, 0.18, 0.7); // limit abs speed
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

        } while (opModeIsActive() && (Math.abs(error) > 0.3 && Math.abs(errorP1) > 0.3 && Math.abs(errorP2) > 0.3) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    // pivot using IMU and motors are set to run to position
    public void pivotDst(double turnAngle, double speed) // should add timeout later
    {
        double pivotSpeed;
        double startAngle;
        double curTurnAngle;

        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;
        double timeout = 3.0;

        // run to position mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //       angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        //       startAngle = adjustAngles(angles.firstAngle);
        startAngle = imu.getAngularOrientation().firstAngle;

        // read angle, record in starting angle variable
        // run motor
        // loop, current angle - start angle = error
        // if error is close to 0, then stop motors


        double error = 100;
        double errorP1 = 100;
        double errorP2 = 100;
        double incDst;
        double motSpeed;

        do
        {
            errorP2 = errorP1;
            errorP1 = error;
//            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
//            curTurnAngle = adjustAngles(angles.firstAngle) - startAngle;
            curTurnAngle = imu.getAngularOrientation().firstAngle - startAngle;
            curTurnAngle = adjustAngles(curTurnAngle);
            error =  turnAngle - curTurnAngle;

            motSpeed = Math.abs(error)/100;
            motSpeed =  Range.clip(motSpeed, 0.35, 0.7); // limit abs speed
            incDst = Math.abs(error) / 20;
            incDst = Range.clip(incDst, 0.18, 5); // limit incDst
            incDst = incDst * Math.signum(error);

            // ccw rotation for pos pivot angle
            newTargetFL = motorFrontLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * incDst);
            newTargetFR = motorFrontRight.getCurrentPosition() - (int) (COUNTS_PER_INCH * incDst);
            newTargetBL = motorBackLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * incDst);
            newTargetBR = motorBackRight.getCurrentPosition() - (int) (COUNTS_PER_INCH * incDst);

            motorFrontLeft.setTargetPosition(newTargetFL);
            motorFrontRight.setTargetPosition(newTargetFR);
            motorBackLeft.setTargetPosition(newTargetBL);
            motorBackRight.setTargetPosition(newTargetBR);

            runtime.reset();
            motorFrontLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorBackLeft.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));

            // wait until the motors reach the position
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy()));

            if (Math.abs(error) < 3) {
                sleep(60);
            }

            telemetry.log().add(String.format("StartAngle: %f, CurAngle: %f, error: %f", startAngle, curTurnAngle, error));

        } while (opModeIsActive() && (Math.abs(error) > 0.5 || Math.abs(errorP1) > 0.5 || Math.abs(errorP2) > 0.5) );
//        } while (opModeIsActive() && (Math.abs(incDst) > 0.2 ) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void PushCapBall() throws InterruptedException
    {
        forwards(45, 0.7, 3);
        sleep(500);
        forwards(5, 0.7, 3);
    }

    public void parkCornerVortex() throws InterruptedException
    {
        forwards(30, 0.7, 3);
        if (isRedTeam)
        {
            moveAngle(20, -90, 0.8, 3); // horizontal left (20 inches)
        }
        else
        {
            moveAngle(20, 90, 0.8, 3); // horizontal right
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
    d. right     +        0      0        +


    */