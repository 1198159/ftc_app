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

//@Autonomous(name="Master Autonomous", group = "Swerve")
// @Disabled

abstract class MasterAutonomous extends MasterOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime autoRuntime = new ElapsedTime();

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

    //double teamAngle;
    double pivotAngle; // will be 60, -60 depending on what team you're on
    double targetAngle; // the Vuforia angle to align to depending on what team you're on

    float[] targetPos = {1524, mmFTCFieldWidth}; // target position x and y with an origin right between the driver stations
    VuforiaNavigation VuforiaNav = new VuforiaNavigation();

    double Kmove = 1.0f/1200.0f; // speed is proportional to error
    double Kpivot = 1.0f/150.0f;

    double TOL = 100;
    double TOL_ANGLE = 5;

    double VUFORIA_TOL = 20;
    double VUFORIA_TOL_ANGLE = 1;

    double MINSPEED = 0.25;


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

        motorFrontLeft.setMaxSpeed(2700);
        motorFrontRight.setMaxSpeed(2700);
        motorBackLeft.setMaxSpeed(2700);
        motorBackRight.setMaxSpeed(2700);

        //Set up telemetry data
        // We show the log in oldest-to-newest order, as that's better for poetry
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.OLDEST_FIRST);
        // We can control the number of lines shown in the log
        telemetry.log().setCapacity(4);
        //configureDashboard();
    }

    // drive forwards/backwards/horizontal left and right function
    public void move(double x, double y, double speed, double timeout) throws InterruptedException
    {
        //MINSPEED = 0.25;
        //Kmove = 1.0/1200.0;
        //TOL = 20;

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

        /*
        newTargetFL = motorFrontLeft.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * forwardMm) + (int) Math.round(COUNTS_PER_MM * horiMm * 1.414);
        newTargetFR = motorFrontRight.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * forwardMm) - (int) Math.round(COUNTS_PER_MM * horiMm * 1.414);
        newTargetBL = motorBackLeft.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * forwardMm) - (int) Math.round(COUNTS_PER_MM * horiMm * 1.414);
        newTargetBR = motorBackRight.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * forwardMm) + (int) Math.round(COUNTS_PER_MM * horiMm * 1.414);
        */

        newTargetFL = motorFrontLeft.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) + (int) Math.round(COUNTS_PER_MM * x * 1.0);
        newTargetFR = motorFrontRight.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) - (int) Math.round(COUNTS_PER_MM * x * 1.0);
        newTargetBL = motorBackLeft.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) - (int) Math.round(COUNTS_PER_MM * x * 1.0);
        newTargetBR = motorBackRight.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) + (int) Math.round(COUNTS_PER_MM * x * 1.0);

        runtime.reset(); // used for timeout

        // wait until the motors reach the position
        do
        {
            errorFL = newTargetFL - motorFrontLeft.getCurrentPosition();
            speedFL = Math.abs(errorFL * Kmove);
            speedFL = Range.clip(speedFL, MINSPEED, speed);
            speedFL = speedFL * Math.signum(errorFL);

            errorFR = newTargetFR - motorFrontRight.getCurrentPosition();
            speedFR = Math.abs(errorFR * Kmove);
            speedFR = Range.clip(speedFR, MINSPEED, speed);
            speedFR = speedFR * Math.signum(errorFR);

            errorBL = newTargetBL - motorBackLeft.getCurrentPosition();
            speedBL = Math.abs(errorBL * Kmove);
            speedBL = Range.clip(speedBL, MINSPEED, speed);
            speedBL = speedBL * Math.signum(errorBL);

            errorBR = newTargetBR - motorBackRight.getCurrentPosition();
            speedBR = Math.abs(errorBR * Kmove);
            speedBR = Range.clip(speedBR, MINSPEED, speed);
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
            VuforiaNav.getLocation(targetIndex); // update target location and angle
            //CodeReview: sometimes getLocation returns null. Sometimes Vuforia.lastLocation might be null. Does your code handle that case gracefully?

            do
            {
                VuforiaNav.getLocation(targetIndex); // update target location and angle
                idle();
            }
            while (VuforiaNav.lastLocation == null);

            // now extract the angle out of "get location", and stores your location
            curTurnAngle = Orientation.getOrientation(VuforiaNav.lastLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            curTurnAngle = adjustAngles(curTurnAngle);
            error =  targetAngle - curTurnAngle;
            pivotSpeed = speed * Math.abs(error) * Kpivot;
            pivotSpeed = Range.clip(pivotSpeed, 0.2, speed); // limit abs speed
            pivotSpeed = pivotSpeed * Math.signum(error); // set the sign of speed


            pivot(error, pivotSpeed);

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

        } while (opModeIsActive() && (Math.abs(error) > TOL_ANGLE));    //&& Math.abs(errorP1) > 0.3 && Math.abs(errorP2) > 0.3) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }


    // move the robot hori. sideways to align with image target
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
                VuforiaNav.getLocation(targetIndex); // update target location and angle
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


    // a combination of both the align and pivot function (WITHOUT VUFORIA)
    // angle has to be small otherwise won't work, this function moves and pivots robot
    public void pivotMove(double x, double y, double pivotAngle, double speed, double timeout)
    {
        MINSPEED = 0.35;
        // run with encoder mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        final double ROBOT_DIAMETER_MM = 27.6 * 25.4;   // diagonal 17.6 inch FL to BR and FR to BL
        pivotDst = (int) ((pivotAngle / 360.0) * ROBOT_DIAMETER_MM * 3.1415 * COUNTS_PER_MM);

/*
        newTargetFL = motorFrontLeft.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * (x * 1.414))
                + (int) Math.round(COUNTS_PER_MM * (y)) + pivotDst;
        newTargetFR = motorFrontRight.getCurrentPosition() - (int) Math.round(COUNTS_PER_MM * (x * 1.414))
                + (int) Math.round(COUNTS_PER_MM * (y)) - pivotDst;
        newTargetBL = motorBackLeft.getCurrentPosition() - (int) Math.round(COUNTS_PER_MM * (x * 1.414))
                + (int) Math.round(COUNTS_PER_MM * (y)) + pivotDst;
        newTargetBR = motorBackRight.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * (x * 1.414))
                + (int) Math.round(COUNTS_PER_MM * (y)) - pivotDst;
*/

        newTargetFL = motorFrontLeft.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * (x * 1.1))
                + (int) Math.round(COUNTS_PER_MM * (y)) + pivotDst;
        newTargetFR = motorFrontRight.getCurrentPosition() - (int) Math.round(COUNTS_PER_MM * (x * 1.1))
                + (int) Math.round(COUNTS_PER_MM * (y)) - pivotDst;
        newTargetBL = motorBackLeft.getCurrentPosition() - (int) Math.round(COUNTS_PER_MM * (x * 1.1))
                + (int) Math.round(COUNTS_PER_MM * (y)) + pivotDst;
        newTargetBR = motorBackRight.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * (x * 1.1))
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
            pivotSpeed = errorAngle * Kpivot;
            pivotSpeed = Range.clip(pivotSpeed, -0.3, 0.3); // limit max pivot speed
            // pivotSpeed is added to each motor's movement speed

            errorFL = newTargetFL - motorFrontLeft.getCurrentPosition();
            speedFL = Kmove * errorFL;  // movement speed proportional to error
            speedFL += pivotSpeed;  // combine movement and pivot speeds
            speedAbsFL = Math.abs(speedFL);
            speedAbsFL = Range.clip(speedAbsFL, MINSPEED, speed);  // clip abs(speed)
            speedFL = speedAbsFL * Math.signum(speedFL);  // set sign of speed

            errorFR = newTargetFR - motorFrontRight.getCurrentPosition();
            speedFR = Kmove * errorFR;
            speedFR -= pivotSpeed;  // combine movement and pivot speeds
            speedAbsFR = Math.abs(speedFR);
            speedAbsFR = Range.clip(speedAbsFR, MINSPEED, speed);  // clip abs(speed)
            speedFR = speedAbsFR * Math.signum(speedFR);

            errorBL = newTargetBL - motorBackLeft.getCurrentPosition();
            speedBL = Kmove * errorBL;
            speedBL += pivotSpeed;  // combine movement and pivot speeds
            speedAbsBL = Math.abs(speedBL);
            speedAbsBL = Range.clip(speedAbsBL, MINSPEED, speed);  // clip abs(speed)
            speedBL = speedAbsBL * Math.signum(speedBL);

            errorBR = newTargetBR - motorBackRight.getCurrentPosition();
            speedBR = Kmove * errorBR;
            speedBR -= pivotSpeed;  // combine movement and pivot speeds
            speedAbsBR = Math.abs(speedBR);
            speedAbsBR = Range.clip(speedAbsBR, MINSPEED, speed);
            speedBR = speedAbsBR * Math.signum(speedBR);

/*
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
*/
            motorFrontLeft.setPower(speedFL);
            motorFrontRight.setPower(speedFR);
            motorBackLeft.setPower(speedBL);
            motorBackRight.setPower(speedBR);

            idle();
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


    // a combination of both the align and pivot function (WITHOUT VUFORIA)
    // angle has to be small otherwise won't work, this function moves and pivots robot
    public void moveAverage(double x, double y, double pivotAngle, double speed, double timeout)
    {
        // run with encoder mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        final double ROBOT_DIAMETER_MM = 27.6 * 25.4;   // diagonal 17.6 inch FL to BR and FR to BL
        pivotDst = (int) ((pivotAngle / 360.0) * ROBOT_DIAMETER_MM * 3.1415 * COUNTS_PER_MM);

        newTargetFL = motorFrontLeft.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * (x * 1.1))
                + (int) Math.round(COUNTS_PER_MM * (y)) + pivotDst;
        newTargetFR = motorFrontRight.getCurrentPosition() - (int) Math.round(COUNTS_PER_MM * (x * 1.1))
                + (int) Math.round(COUNTS_PER_MM * (y)) - pivotDst;
        newTargetBL = motorBackLeft.getCurrentPosition() - (int) Math.round(COUNTS_PER_MM * (x * 1.1))
                + (int) Math.round(COUNTS_PER_MM * (y)) + pivotDst;
        newTargetBR = motorBackRight.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * (x * 1.1))
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
            pivotSpeed = errorAngle * Kpivot;
            pivotSpeed = Range.clip(pivotSpeed, -0.3, 0.3); // limit max pivot speed
            // pivotSpeed is added to each motor's movement speed

            errorFL = newTargetFL - motorFrontLeft.getCurrentPosition();
            speedFL = Kmove * errorFL;  // movement speed proportional to error
            speedFL += pivotSpeed;  // combine movement and pivot speeds
            speedAbsFL = Math.abs(speedFL);
            speedAbsFL = Range.clip(speedAbsFL, MINSPEED, speed);  // clip abs(speed)
            speedFL = speedAbsFL * Math.signum(speedFL);  // set sign of speed

            errorFR = newTargetFR - motorFrontRight.getCurrentPosition();
            speedFR = Kmove * errorFR;
            speedFR -= pivotSpeed;  // combine movement and pivot speeds
            speedAbsFR = Math.abs(speedFR);
            speedAbsFR = Range.clip(speedAbsFR, MINSPEED, speed);  // clip abs(speed)
            speedFR = speedAbsFR * Math.signum(speedFR);

            errorBL = newTargetBL - motorBackLeft.getCurrentPosition();
            speedBL = Kmove * errorBL;
            speedBL += pivotSpeed;  // combine movement and pivot speeds
            speedAbsBL = Math.abs(speedBL);
            speedAbsBL = Range.clip(speedAbsBL, MINSPEED, speed);  // clip abs(speed)
            speedBL = speedAbsBL * Math.signum(speedBL);

            errorBR = newTargetBR - motorBackRight.getCurrentPosition();
            speedBR = Kmove * errorBR;
            speedBR -= pivotSpeed;  // combine movement and pivot speeds
            speedAbsBR = Math.abs(speedBR);
            speedAbsBR = Range.clip(speedAbsBR, MINSPEED, speed);
            speedBR = speedAbsBR * Math.signum(speedBR);

            motorFrontLeft.setPower(speedFL);
            motorFrontRight.setPower(speedFR);
            motorBackLeft.setPower(speedBL);
            motorBackRight.setPower(speedBR);

            idle();
        }
        while ( (opModeIsActive()) &&
                (runtime.seconds() < timeout) &&
                (
                        ( (Math.abs(errorFL) > TOL) && (Math.abs(errorFR) > TOL) && (Math.abs(errorBL) > TOL) && (Math.abs(errorBR) > TOL) )
                        || (Math.abs(errorAngle) > TOL_ANGLE)
                )
                );

        // stop the motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }


    // a combination of both the align and pivot function (WITH VUFORIA) using pivot move
    public void alignPivotVuforia(double speed, double distAwayX, double distAwayY, double timeout)
    {
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
            while (!VuforiaNav.isVisible(targetIndex) && opModeIsActive())
                idle();
            VuforiaNav.lastLocation = null;
            do
            {
                VuforiaNav.getLocation(targetIndex); // update target location and angle
                idle();
            }
            while (VuforiaNav.lastLocation == null && opModeIsActive());

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
            robotErrorY -= distAwayY;
            robotErrorX += distAwayX;

            telemetry.log().add("executing");
            telemetry.update();
// calls pivot move function here
            //pivotMove(robotErrorX, robotErrorY, errorAngle, speed, timeout); // speed, 3 second timeout
            moveAverage(robotErrorX, robotErrorY, errorAngle, speed, timeout); // speed, 3 second timeout
            telemetry.log().add("done");
            telemetry.update();

            runtime.reset();
            telemetry.log().add(String.format("%d %.2f %.2f %.2f", loopCount, robotErrorX, robotErrorY, errorAngle)); // display each motor error as well
            telemetry.update();
            loopCount++;

            // stop the motors
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            sleep(200);

            // error is in mm
        } while ( opModeIsActive() && ( Math.abs(robotErrorX) > VUFORIA_TOL || Math.abs(robotErrorY) > VUFORIA_TOL
                || errorAngle > VUFORIA_TOL_ANGLE ) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }


    // pivot using IMU, it moves in a staccato-like manner
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
            curTurnAngle = imu.getAngularOrientation().firstAngle - startAngle;
            curTurnAngle = adjustAngles(curTurnAngle);
            error =  turnAngle - curTurnAngle;
            pivotSpeed = speed * Math.abs(error) * Kpivot;
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

        } while (opModeIsActive() && (Math.abs(error) > TOL_ANGLE || Math.abs(errorP1) > TOL_ANGLE) );

        // stop motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }


    public void pivotDetectTarget (double span, int numIterations) throws InterruptedException
    {
        double angle = 0.0;
        int i;
        for (i = 0; i < numIterations; i++)
        {
            // allow Vuforia to catch up
            sleep(300);
            if (VuforiaNav.isVisible(targetIndex))
            {
                return;
            }
            angle += span / (double) numIterations;
            //pivot(angle, 0.5);
            PivotForSeconds(0.5, 200);
        }
        //pivot(-span, 0.7);
        PivotForSeconds(-0.5, 800);
        angle = 0.0;
        sleep(300);
        for (i = 0; i < numIterations; i++)
        {
            sleep(300);
            if (VuforiaNav.isVisible(targetIndex))
            {
                return;
            }
            angle -= span / (double) numIterations;
            //pivot(angle, 0.5);
            PivotForSeconds(-0.5, 200);
        }
    }


    public void CornerVortexOption() throws InterruptedException
    {
        // CORNER VORTEX OPTION
        if (isRedTeam) pivot(-100, 0.6); // if red team, pivot right
        else pivot(100, 0.6);

        DriveForSeconds(-0.6, 2500); // drive for 2.5 seconds
    }


    // this method drives for seconds, and it can only drive forwards or backwards
    public void DriveForSeconds(double speed, int milliSeconds) throws InterruptedException
    {
        // turn on power
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(speed);
        // let it run for x seconds
        pause(milliSeconds);
        // stop the motors after two seconds
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    // this method drives for seconds, and it can only pivot
    public void PivotForSeconds(double speed, int milliSeconds) throws InterruptedException
    {
        // turn on power
        motorFrontLeft.setPower(speed);
        motorFrontRight.setPower(-speed);
        motorBackLeft.setPower(speed);
        motorBackRight.setPower(-speed);
        // let it run for x seconds
        pause(milliSeconds);
        // stop the motors after two seconds
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }


    public void CenterVortextOption() throws InterruptedException
    {
        // CENTER VORTEX OPTION
        if (isRedTeam) pivot(50, 0.6);
        else pivot(-50, 0.6);
        //forwards(-30, 0, 0.6, 3);
        move(0, 762, 0.6, 3);
    }

    // pushes the cap ball and parks on the center vortex
    public void PushCapBall() throws InterruptedException
    {
        //forwards(45, 0, 0.7, 3);
        move(0, 1143, 0.7, 3);
        pause(500);
        //forwards(5, 0, 0.7, 3);
        move(0, 127, 0.7, 3);
    }


    // parks parks on corner vortex
    public void parkCornerVortex() throws InterruptedException
    {
        //forwards(30, 0, 0.7, 3);
        move(0, 762, 0.7, 3);
        if (isRedTeam)
        {
            move(-508, 0, 0.8, 3);
            //forwards(0, -20, 0.8, 3); // horizontal left (20 inches)
        }
        else
        {
            //forwards(0, 20, 0.8, 3); // horizontal right
            move(508, 0, 0.8, 3);
        }
    }

    public void parkCornerVortexAfterBeacons() throws InterruptedException
    {
        move(0, -300, 0.5, 3); // back up
        if (isRedTeam)
        {
            pivot(-45, 0.8);
        }
        else
        {
            pivot(45, 0.8);
        }
        move(0, -1524, 0.8, 3); // move back 60 inches
    }

    public void shootParticlesAfterBeacons() throws InterruptedException
    {

        TOL_ANGLE = 3.0;
        VUFORIA_TOL_ANGLE = 3.0;
        Kpivot = 1/140.0;
        MINSPEED = 0.35;

        motorLauncher.setPower(0.5);
        if (isRedTeam)
        {
            pivot(-41, 0.8);
        }
        else
        {
            pivot(50, 0.8);
        }

        //pause(1000);
        servoParticle.setPosition(0.8);
        pause(300);
        servoParticle.setPosition(0.0);
        pause(900);
        servoParticle.setPosition(0.8);
        pause(300);
        servoParticle.setPosition(0.0);
        motorLauncher.setPower(0.0);
    }

    public void PushButton() throws InterruptedException
    {
        pause(70);
        telemetry.addData("Path", "pushing button");
        telemetry.update();
        move(0, 300, 0.25, 3); // push the button, used to be 361mm forwards
        telemetry.log().add(String.format("pushed button"));
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