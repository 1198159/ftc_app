package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Autonomous(name="Master Autonomous", group = "Swerve")
// @Disabled

abstract class MasterAutonomous extends MasterOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    public ElapsedTime autoRuntime = new ElapsedTime();
    VuforiaDetection VuforiaDetect = new VuforiaDetection();

    final float mmPerInch = 25.4f;
    final float mmBotWidth = 18 * mmPerInch; // the robot width
    final float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    boolean isLogging = true;
    boolean isRedTeam; // are you on red team? (if not, you're blue)
    boolean isStartingPosOne;  // are you on starting position one? (if not, you're on position two)

    // speed is proportional to error
    double Kmove = 1.0f/1200.0f;
    double Kpivot = 1.0f/150.0f;

    double TOL = 100.0;
    double TOL_ANGLE = 5;

    double MINSPEED = 0.25;
    double PIVOT_MINSPEED = 0.2;


    // VARIABLES FOR MOVE/ALIGN METHODS
    int pivotDst;

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

    double avgDistError;
    double avgSpeed;
    double speedAbsAvg;

    double refAngle;


    public void initializeRobot()
    {
        super.initializeHardware(); // call master op mode's init method
        VuforiaDetect.initVuforia();

        // zero the motor controllers before running, don't know if motors start out at zero
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        newTargetFL = motorFL.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) + (int) Math.round(COUNTS_PER_MM * x * 1.15);
        newTargetFR = motorFR.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) - (int) Math.round(COUNTS_PER_MM * x * 1.15);
        newTargetBL = motorBL.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) - (int) Math.round(COUNTS_PER_MM * x * 1.15);
        newTargetBR = motorBR.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y) + (int) Math.round(COUNTS_PER_MM * x * 1.15);

        runtime.reset(); // used for timeout

        // wait until the motors reach the position
        do
        {
            errorFL = newTargetFL + motorFL.getCurrentPosition();
            speedFL = Math.abs(errorFL * Kmove);
            speedFL = Range.clip(speedFL, MINSPEED, speed);
            speedFL = speedFL * Math.signum(errorFL);

            errorFR = newTargetFR + motorFR.getCurrentPosition();
            speedFR = Math.abs(errorFR * Kmove);
            speedFR = Range.clip(speedFR, MINSPEED, speed);
            speedFR = speedFR * Math.signum(errorFR);

            errorBL = newTargetBL + motorBL.getCurrentPosition();
            speedBL = Math.abs(errorBL * Kmove);
            speedBL = Range.clip(speedBL, MINSPEED, speed);
            speedBL = speedBL * Math.signum(errorBL);

            errorBR = newTargetBR + motorBR.getCurrentPosition();
            speedBR = Math.abs(errorBR * Kmove);
            speedBR = Range.clip(speedBR, MINSPEED, speed);
            speedBR = speedBR * Math.signum(errorBR);

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(speedBL);
            motorBR.setPower(speedBR);

            idle();
        }
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (Math.abs(errorFL) > TOL || Math.abs(errorFR) > TOL || Math.abs(errorBL) > TOL || Math.abs(errorBR) > TOL));

        // stop the motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }


    // a combination of both the align and pivot function (WITHOUT VUFORIA)
    // angle has to be small otherwise won't work, this function moves and pivots robot
    public void pivotMove(double x, double y, double pivotAngle, double speed, double timeout)
    {
        final double XSCALE = 1.1;
        MINSPEED = 0.35;
        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final double ROBOT_DIAMETER_MM = 27.6 * 25.4;   // diagonal 17.6 inch FL to BR and FR to BL
        pivotDst = (int) ((pivotAngle / 360.0) * ROBOT_DIAMETER_MM * 3.1415 * COUNTS_PER_MM);

        int xTarget = (int) Math.round(COUNTS_PER_MM * (x * XSCALE));
        int yTarget = (int) Math.round(COUNTS_PER_MM * (y));
        newTargetFL = motorFL.getCurrentPosition() + xTarget + yTarget + pivotDst;
        newTargetFR = motorFR.getCurrentPosition() - xTarget + yTarget - pivotDst;
        newTargetBL = motorBL.getCurrentPosition() - xTarget + yTarget + pivotDst;
        newTargetBR = motorBR.getCurrentPosition() + xTarget + yTarget - pivotDst;

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

            errorFL = newTargetFL - motorFL.getCurrentPosition();
            speedFL = Kmove * errorFL;  // movement speed proportional to error
            speedFL += pivotSpeed;  // combine movement and pivot speeds
            speedAbsFL = Math.abs(speedFL);
            speedAbsFL = Range.clip(speedAbsFL, MINSPEED, speed);  // clip abs(speed)
            speedFL = speedAbsFL * Math.signum(speedFL);  // set sign of speed

            errorFR = newTargetFR - motorFR.getCurrentPosition();
            speedFR = Kmove * errorFR;
            speedFR -= pivotSpeed;  // combine movement and pivot speeds
            speedAbsFR = Math.abs(speedFR);
            speedAbsFR = Range.clip(speedAbsFR, MINSPEED, speed);  // clip abs(speed)
            speedFR = speedAbsFR * Math.signum(speedFR);

            errorBL = newTargetBL - motorFR.getCurrentPosition();
            speedBL = Kmove * errorBL;
            speedBL += pivotSpeed;  // combine movement and pivot speeds
            speedAbsBL = Math.abs(speedBL);
            speedAbsBL = Range.clip(speedAbsBL, MINSPEED, speed);  // clip abs(speed)
            speedBL = speedAbsBL * Math.signum(speedBL);

            errorBR = newTargetBR - motorFR.getCurrentPosition();
            speedBR = Kmove * errorBR;
            speedBR -= pivotSpeed;  // combine movement and pivot speeds
            speedAbsBR = Math.abs(speedBR);
            speedAbsBR = Range.clip(speedAbsBR, MINSPEED, speed);
            speedBR = speedAbsBR * Math.signum(speedBR);

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(speedBL);
            motorBR.setPower(speedBR);

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
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }


    // a combination of both the align and pivot function (WITHOUT VUFORIA)
    // angle has to be small otherwise won't work, this function moves and pivots robot
    public void moveAverage(double x, double y, double pivotAngle, double speed, double timeout)
    {
        final double XSCALE = 1.1;
        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        final double ROBOT_DIAMETER_MM = 21.5 * 25.4;   // diagonal 17.6 inch FL to BR and FR to BL
        pivotDst = (int) ((pivotAngle / 360.0) * ROBOT_DIAMETER_MM * 3.1415 * COUNTS_PER_MM);

        int xTarget = (int) Math.round(COUNTS_PER_MM * (x * XSCALE));
        int yTarget = (int) Math.round(COUNTS_PER_MM * (y));
        newTargetFL = -motorFL.getCurrentPosition() + xTarget + yTarget + pivotDst;
        newTargetFR = -motorFR.getCurrentPosition() - xTarget + yTarget - pivotDst;
        newTargetBL = -motorBL.getCurrentPosition() - xTarget + yTarget + pivotDst;
        newTargetBR = -motorBR.getCurrentPosition() + xTarget + yTarget - pivotDst;

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

            errorFL = newTargetFL - motorFL.getCurrentPosition();
            speedFL = Kmove * errorFL; // movement speed proportional to error
            speedFL += pivotSpeed;  // combine movement and pivot speeds
            speedAbsFL = Math.abs(speedFL);
            speedAbsFL = Range.clip(speedAbsFL, MINSPEED, speed);  // clip abs(speed)
            speedFL = speedAbsFL * Math.signum(speedFL);  // set sign of speed

            errorFR = newTargetFR - motorFR.getCurrentPosition();
            speedFR = Kmove * errorFR;
            speedFR -= pivotSpeed;  // combine movement and pivot speeds
            speedAbsFR = Math.abs(speedFR);
            speedAbsFR = Range.clip(speedAbsFR, MINSPEED, speed);  // clip abs(speed)
            speedFR = speedAbsFR * Math.signum(speedFR);

            errorBL = newTargetBL - motorBL.getCurrentPosition();
            speedBL = Kmove * errorBL;
            speedBL += pivotSpeed;  // combine movement and pivot speeds
            speedAbsBL = Math.abs(speedBL);
            speedAbsBL = Range.clip(speedAbsBL, MINSPEED, speed);  // clip abs(speed)
            speedBL = speedAbsBL * Math.signum(speedBL);

            errorBR = newTargetBR - motorBR.getCurrentPosition();
            speedBR = Kmove * errorBR;
            speedBR -= pivotSpeed;  // combine movement and pivot speeds
            speedAbsBR = Math.abs(speedBR);
            speedAbsBR = Range.clip(speedAbsBR, MINSPEED, speed);
            speedBR = speedAbsBR * Math.signum(speedBR);

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(speedBL);
            motorBR.setPower(speedBR);

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
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    // a combination of both the align and pivot function (WITHOUT VUFORIA)
    // angle has to be small otherwise won't work, this function moves and pivots robot
    public void moveMaintainHeading(double x, double y, double pivotAngle, double refAngle, double speed, double timeout)
    {
        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        curTurnAngle = imu.getAngularOrientation().firstAngle - refAngle;
        curTurnAngle = adjustAngles(curTurnAngle);
        errorAngle =  pivotAngle - curTurnAngle;

        final double ROBOT_DIAMETER_MM = 21.6;   // diagonal 17.6 inch FL to BR and FR to BL
        pivotDst = (int) ((errorAngle / 360.0) * ROBOT_DIAMETER_MM * 3.1415 * COUNTS_PER_MM);

        final double XSCALE = 1.1;

        int xTarget = (int) Math.round(COUNTS_PER_MM * (x * XSCALE));
        int yTarget = (int) Math.round(COUNTS_PER_MM * (y));
        newTargetFL = motorFL.getCurrentPosition() + xTarget + yTarget + pivotDst;
        newTargetFR = motorFR.getCurrentPosition() - xTarget + yTarget - pivotDst;
        newTargetBL = motorBL.getCurrentPosition() - xTarget + yTarget + pivotDst;
        newTargetBR = motorBR.getCurrentPosition() + xTarget + yTarget - pivotDst;

        runtime.reset(); // reset timer, which is used for loop timeout below

        // wait until the motors reach the position
        // adjust robot angle during movement by adjusting speed of motors
        do
        {
            // read the real current angle and compute error compared to ref angle
            curTurnAngle = imu.getAngularOrientation().firstAngle - refAngle;
            curTurnAngle = adjustAngles(curTurnAngle);
            errorAngle =  pivotAngle - curTurnAngle;
            pivotSpeed = errorAngle * Kpivot;
            pivotSpeed = Range.clip(pivotSpeed, -0.3, 0.3); // limit max pivot speed
            // pivotSpeed is added to each motor's movement speed

            errorFL = newTargetFL - motorFL.getCurrentPosition();
            speedFL = Kmove * errorFL;  // movement speed proportional to error
            speedAbsFL = Math.abs(speedFL);
            // clip abs(speed) MAX speed minus 0.3 to leave room for pivot factor
            speedAbsFL = Range.clip(speedAbsFL, MINSPEED, speed - 0.3);
            speedFL = speedAbsFL * Math.signum(speedFL);  // set sign of speed
            speedFL += pivotSpeed;  // combine movement and pivot speeds

            errorFR = newTargetFR - motorFR.getCurrentPosition();
            speedFR = Kmove * errorFR;
            speedAbsFR = Math.abs(speedFR);
            speedAbsFR = Range.clip(speedAbsFR, MINSPEED, speed - 0.3);  // clip abs(speed)
            speedFR = speedAbsFR * Math.signum(speedFR);
            speedFR -= pivotSpeed;  // combine movement and pivot speeds

            errorBL = newTargetBL - motorBL.getCurrentPosition();
            speedBL = Kmove * errorBL;
            speedAbsBL = Math.abs(speedBL);
            speedAbsBL = Range.clip(speedAbsBL, MINSPEED, speed - 0.3);  // clip abs(speed)
            speedBL = speedAbsBL * Math.signum(speedBL);
            speedBL += pivotSpeed;  // combine movement and pivot speeds

            errorBR = newTargetBR - motorBR.getCurrentPosition();
            speedBR = Kmove * errorBR;
            speedAbsBR = Math.abs(speedBR);
            speedAbsBR = Range.clip(speedAbsBR, MINSPEED, speed - 0.3);
            speedBR = speedAbsBR * Math.signum(speedBR);
            speedBR -= pivotSpeed;  // combine movement and pivot speeds

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(speedBL);
            motorBR.setPower(speedBR);

            avgDistError = (Math.abs(errorFL) + Math.abs(errorFR) + Math.abs(errorBL) + Math.abs(errorBR)) / 4.0;
            idle();
        }
        while ( (opModeIsActive()) &&
                (runtime.seconds() < timeout) &&
                (
                     //   ( (Math.abs(errorFL) > TOL) && (Math.abs(errorFR) > TOL) && (Math.abs(errorBL) > TOL) && (Math.abs(errorBR) > TOL) )
                        avgDistError > TOL
                                || (Math.abs(errorAngle) > TOL_ANGLE)
                )
                );

        // stop the motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }


    public void moveKeepHeading(double x, double y, double pivotAngle, double refAngle, double speed, double timeout)
    {
        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double avgDistError;

        curTurnAngle = imu.getAngularOrientation().firstAngle - refAngle;
        curTurnAngle = adjustAngles(curTurnAngle);
        errorAngle =  pivotAngle - curTurnAngle;

        final double ROBOT_DIAMETER_MM = 22.6 * 25.4;   // diagonal 17.6 inch FL to BR and FR to BL
        pivotDst = (int) ((errorAngle / 360.0) * ROBOT_DIAMETER_MM * 3.1415 * COUNTS_PER_MM);

        final double XSCALE = 1.1;

        int xTarget = (int) Math.round(COUNTS_PER_MM * (x * XSCALE));
        int yTarget = (int) Math.round(COUNTS_PER_MM * (y));
        newTargetFL = motorFL.getCurrentPosition() + xTarget + yTarget + pivotDst;
        newTargetFR = motorFR.getCurrentPosition() - xTarget + yTarget - pivotDst;
        newTargetBL = motorBL.getCurrentPosition() - xTarget + yTarget + pivotDst;
        newTargetBR = motorBR.getCurrentPosition() + xTarget + yTarget - pivotDst;

        runtime.reset(); // reset timer, which is used for loop timeout below

        // wait until the motors reach the position
        // adjust robot angle during movement by adjusting speed of motors
        do
        {
            // read the real current angle and compute error compared to ref angle
            curTurnAngle = imu.getAngularOrientation().firstAngle - refAngle;
            curTurnAngle = adjustAngles(curTurnAngle);
            errorAngle =  pivotAngle - curTurnAngle;
            pivotSpeed = errorAngle * Kpivot;
            pivotSpeed = Range.clip(pivotSpeed, -0.3, 0.3); // limit max pivot speed
            // pivotSpeed is added to each motor's movement speed

            errorFL = newTargetFL - motorFL.getCurrentPosition();
            errorFR = newTargetFR - motorFR.getCurrentPosition();
            errorBL = newTargetBL - motorBL.getCurrentPosition();
            errorBR = newTargetBR - motorBR.getCurrentPosition();
            avgDistError = (errorFL + errorFR + errorBL + errorBR) / 4.0;
            avgSpeed = Kmove * avgDistError;
            speedAbsAvg = Range.clip(Math.abs(avgSpeed), MINSPEED, speed - 0.3);
            avgSpeed = speedAbsAvg * Math.signum(avgSpeed);  // set sign of speed

            speedFL = avgSpeed + pivotSpeed; // combine movement and pivot speeds
            speedFR = avgSpeed - pivotSpeed;
            speedBL = avgSpeed + pivotSpeed;
            speedBR = avgSpeed - pivotSpeed;

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(speedBL);
            motorBR.setPower(speedBR);

            if (isLogging) telemetry.log().add(String.format("avgDistError: %f , EA: %f", avgDistError, errorAngle));

            idle();
        }
        while ( (opModeIsActive()) &&
                (runtime.seconds() < timeout) &&
                (
                        //   ( (Math.abs(errorFL) > TOL) && (Math.abs(errorFR) > TOL) && (Math.abs(errorBL) > TOL) && (Math.abs(errorBR) > TOL) )
                        Math.abs(avgDistError) > TOL
                                || (Math.abs(errorAngle) > TOL_ANGLE)
                )
                );

        if (isLogging) telemetry.log().add(String.format("avgDistError: %f , EA: %f", avgDistError, errorAngle));
        telemetry.update();

        // stop the motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }


    // pivot using IMU
    public void pivot(double turnAngle, double speed)
    {
        double pivotSpeed;
        double startAngle;
        double curTurnAngle;

        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            pivotSpeed = Math.abs(error) * Kpivot;
            pivotSpeed = Range.clip(pivotSpeed, PIVOT_MINSPEED, speed); // limit abs speed
            pivotSpeed = pivotSpeed * Math.signum(error); // set the sign of speed

            // positive angle means CCW rotation
            motorFL.setPower(pivotSpeed);
            motorFR.setPower(-pivotSpeed);
            motorBL.setPower(pivotSpeed);
            motorBR.setPower(-pivotSpeed);

            // allow some time for IMU to catch up
            if (Math.abs(error) < 2)
            {
                sleep(15);
                // stop motors
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                sleep(150);
            }

            if (isLogging) telemetry.log().add(String.format("StartAngle: %f, CurAngle: %f, error: %f", startAngle, curTurnAngle, error));
            idle();

        } while (opModeIsActive() && (Math.abs(error) > TOL_ANGLE || Math.abs(errorP1) > TOL_ANGLE) );

        // stop motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    // pivot using IMU, but with a reference start angle, but this angle has to be determined (read) before this method is called
    public void pivotWithReference(double targetAngle, double refAngle, double speed)
    {
        double pivotSpeed;
        double currentAngle;
        double errorAngle;

        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // read angle, record in starting angle variable
        // run motor
        // loop, current angle - start angle = error
        // if error is close to 0, stop motors

        do
        {
            currentAngle = imu.getAngularOrientation().firstAngle - refAngle;
            errorAngle = currentAngle - targetAngle;
            pivotSpeed = Math.abs(errorAngle) * Kpivot;
            pivotSpeed = Range.clip(pivotSpeed, PIVOT_MINSPEED, speed); // limit abs speed
            pivotSpeed = pivotSpeed * Math.signum(errorAngle); // set the sign of speed

            // positive angle means CCW rotation
            motorFL.setPower(pivotSpeed);
            motorFR.setPower(-pivotSpeed);
            motorBL.setPower(pivotSpeed);
            motorBR.setPower(-pivotSpeed);

            if (isLogging) telemetry.log().add(String.format("StartAngle: %f, CurAngle: %f, error: %f", refAngle, currentAngle, errorAngle));
            idle();

        } while (opModeIsActive() && (Math.abs(errorAngle) > TOL_ANGLE) );

        // stop motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }



    public void configureDashboard()
    {
        telemetry.addLine()
                .addData("Power | FrontLeft: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorFL.getPower());
                    }
                })
                .addData("Power | FrontRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorFR.getPower());
                    }
                })
                .addData("Power | BackLeft: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBL.getPower());
                    }
                })
                .addData("Power | BackRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBR.getPower());
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