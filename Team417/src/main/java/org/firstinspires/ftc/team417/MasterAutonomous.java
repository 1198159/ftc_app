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
@Autonomous(name="Master Autonomous", group = "Swerve")
// @Disabled

public class MasterAutonomous extends MasterOpMode
{
    // HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    // declare "what team we're on" variables
    boolean isRedTeam;
    boolean isPosOne;
    double startDelay;
    double teamAngle; // will be 60, -60
    float[] targetPos = {1524, mmFTCFieldWidth};
    VuforiaNavigation VuforiaNav = new VuforiaNavigation();


    @Override
    public void runOpMode() throws InterruptedException
    {
        double pivotAngle;
        double targetAngle;
        double startDist; // the distance traveled depending on pos one or two
        // Initialize hardware and other important things
        initializeRobot();

        // allow driver to choose a team
        if (gamepad1.x)
        {
            isRedTeam = true;
        }
        if (gamepad1.b)
        {
            isRedTeam = false;
        }

        // select position one or two, one is closer to the origin
        if (gamepad1.y)
        {
            isPosOne = true;
        }
        if (gamepad1.b)
        {
            isPosOne = false;
        }

        VuforiaNav.initVuforia();
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        telemetry.addData("Path", "InitDone");
        if (isRedTeam) // if team RED
        {
            if (isPosOne)
            {
                // OPTION RED ONE (TOOLS)
                startDelay = 2000;
                pivotAngle = 60; // pivot this amount before aquiring target
                targetAngle = 0; // Vuforia angle
                targetPos[0] = 2743.2f;
                targetPos[1] = mmFTCFieldWidth;
                telemetry.addData("Team: ", "Red 1"); // display what team we're on after choosing with the buttons
            }
            else
            {
                // OPTION RED TWO (GEARS)
                pivotAngle = 60; // pivot this amount before aquiring target
                targetAngle = 0; // Vuforia angle
                targetPos[0] = 1524;
                targetPos[1] = mmFTCFieldWidth;
                telemetry.addData("Team: ", "Red 2"); // display what team we're on after choosing with the buttons
            }

        }
        else // if team BLUE
        {
            if (isPosOne)
            {
                // OPTION BLUE ONE (LEGOS)
                pivotAngle = -60;
                targetAngle = -90;
                targetPos[0] = mmFTCFieldWidth;
                targetPos[1] = 2743.2f;
                telemetry.addData("Team: ", "Blue 1");
            }
            else
            {
                // OPTION BLUE TWO (WHEELS)
                pivotAngle = -60;
                targetAngle = -90;
                targetPos[0] = mmFTCFieldWidth;
                targetPos[1] = 1524;
                telemetry.addData("Team: ", "Blue 2");
            }
        }
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        VuforiaNav.startTracking();
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

// TODO: test these forwards and backwards, etc. tomorrow

        /*forwards(-24, 3, 0.5);  // inches, timeout, speed
        sleep(500);
        pivot(60, 0.7);
        sleep(500);
        moveAngle(20, 45, .8, 3);
        sleep(500);
        pivot(-60, 0.7);
        sleep(500);
        */

       // forwards(-6, 3, 0.2);
       // sleep(500);
       // forwards(-6, 3, 0.2);
       // sleep(500);
       // forwards(6, 3, 0.2);
       // sleep(500);
       // pivotDst(90, 0.6);
       // sleep(1000);
       // pivotDst(-90, 0.6);
        // moveAngle(6, 45, .8, 3);
        // sleep(5000);

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

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLift.setPower(0);

        //Set up telemetry data
        // We show the log in oldest-to-newest order, as that's better for poetry
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        telemetry.log().setCapacity(3);
        //configureDashboard();
    }

    // drive forwards/backwards function
    public void forwards(double forwardInches, double timeout, double speed) throws InterruptedException
    {
        int newTargetFL;
        int newTargetBL;
        int newTargetFR;
        int newTargetBR;


        newTargetFL = motorFrontLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetFR = motorFrontRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetBL = motorBackLeft.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);
        newTargetBR = motorBackRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * forwardInches);

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

        // stop the motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void pivotVuforia (float targetAngle, double speed)
    {
        double error;
        double curTurnAngle;
        double pivotSpeed;

        do
        {
//            angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
//            curTurnAngle = adjustAngles(angles.firstAngle) - startAngle;
            VuforiaNav.getLocation(); // update target location and angle
            // now extract the angle out of "get location", andn stores your location
            curTurnAngle = Orientation.getOrientation(VuforiaNav.lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            curTurnAngle = adjustAngles(curTurnAngle);
            error =  targetAngle - curTurnAngle;
            pivotSpeed = speed * Math.abs(error) / 100;
            pivotSpeed = Range.clip(pivotSpeed, 0.18, 0.7); // limit abs speed
            pivotSpeed = pivotSpeed * Math.signum(error); // set the sign of speed


            // positive angle means CCW rotation
            motorFrontLeft.setPower(-pivotSpeed);
            motorFrontRight.setPower(pivotSpeed);
            motorBackLeft.setPower(-pivotSpeed);
            motorBackRight.setPower(pivotSpeed);

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

            telemetry.log().add(String.format("CurAngle: %f, error: %f", curTurnAngle, error));

        } while (opModeIsActive() && (Math.abs(error) > 0.3));    //&& Math.abs(errorP1) > 0.3 && Math.abs(errorP2) > 0.3) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

    }


    // move the robot hora. sideways to align with image target
    public void alignVuforia (double targetPos, double speed)
    {

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

    public void pivot(double turnAngle, double speed) // should add timeout later
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
            motorFrontLeft.setPower(-pivotSpeed);
            motorFrontRight.setPower(pivotSpeed);
            motorBackLeft.setPower(-pivotSpeed);
            motorBackRight.setPower(pivotSpeed);

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
            newTargetFL = motorFrontLeft.getCurrentPosition() - (int) (COUNTS_PER_INCH * incDst);
            newTargetFR = motorFrontRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * incDst);
            newTargetBL = motorBackLeft.getCurrentPosition() - (int) (COUNTS_PER_INCH * incDst);
            newTargetBR = motorBackRight.getCurrentPosition() + (int) (COUNTS_PER_INCH * incDst);

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

 /* TABLE: Mecanum Drive

                 FL      FR      BL      BR
    rotate ->    +        -      +        -
    rotate <-    -        +      -        +
    forward      +        +      +        +
    backward     -        -      -        -
    left         -        +      +        -
    right        +        -      -        +
    d. left      0        +      +        0
    d. right     +        0      0        +

    */