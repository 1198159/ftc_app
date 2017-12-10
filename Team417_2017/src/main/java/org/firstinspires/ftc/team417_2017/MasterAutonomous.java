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

    boolean isLogging = true;
    boolean isPosLeft;  // are you on starting position one? (if not, you're on position two)

    // speed is proportional to error
    double Kmove = 1.0f/1200.0f;
    double Kpivot = 1.0f/150.0f;

    double TOL = 100.0; // this is in encoder counts, and for our robot with 4 inch wheels, it's 0.285 mm in one encoder count
    double TOL_ANGLE = 5;

    double MINSPEED = 0.25;
    double PIVOT_MINSPEED = 0.2;

    // VARIABLES FOR AUTONOMOUS GG
    int curGGPos;
    int errorMinGG;
    int errorMaxGG;
    int minGGPos = -180; // a bit less than the original starting position of zero (where we start it)
    int maxGGPos = -577; // maxGGPos equals the # rev to close/open GG (13 rev) times 44.4 counts per rev
    double speedGG;
    double KGlyph;

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

    double curTurnAngle;
    double pivotSpeed;
    double errorAngle;

    double avgDistError;
    double avgSpeed;
    double speedAbsAvg;

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
    public void move(double x, double y, double minSpeed, double maxSpeed, double timeout) throws InterruptedException
    {
        newTargetFL = motorFL.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y * SCALE_OMNI) + (int) Math.round(COUNTS_PER_MM * x * SCALE_OMNI);
        newTargetFR = motorFR.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y * SCALE_OMNI) - (int) Math.round(COUNTS_PER_MM * x * SCALE_OMNI);
        newTargetBL = motorBL.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y * SCALE_OMNI) - (int) Math.round(COUNTS_PER_MM * x * SCALE_OMNI);
        newTargetBR = motorBR.getCurrentPosition() + (int) Math.round(COUNTS_PER_MM * y * SCALE_OMNI) + (int) Math.round(COUNTS_PER_MM * x * SCALE_OMNI);

        runtime.reset(); // used for timeout

        // wait until the motors reach the position
        do
        {
            errorFL = newTargetFL - motorFL.getCurrentPosition();
            speedFL = Math.abs(errorFL * Kmove);
            speedFL = Range.clip(speedFL, minSpeed, maxSpeed);
            speedFL = speedFL * Math.signum(errorFL);

            errorFR = newTargetFR - motorFR.getCurrentPosition();
            speedFR = Math.abs(errorFR * Kmove);
            speedFR = Range.clip(speedFR, minSpeed, maxSpeed);
            speedFR = speedFR * Math.signum(errorFR);

            errorBL = newTargetBL - motorBL.getCurrentPosition();
            speedBL = Math.abs(errorBL * Kmove);
            speedBL = Range.clip(speedBL, minSpeed, maxSpeed);
            speedBL = speedBL * Math.signum(errorBL);

            errorBR = newTargetBR - motorBR.getCurrentPosition();
            speedBR = Math.abs(errorBR * Kmove);
            speedBR = Range.clip(speedBR, minSpeed, maxSpeed);
            speedBR = speedBR * Math.signum(errorBR);

            motorFL.setPower(speedFL);
            motorFR.setPower(speedFR);
            motorBL.setPower(speedBL);
            motorBR.setPower(speedBR);

            telemetry.log().add(String.format("errorFL: %d, speedFL: %f" , errorFL, speedFL));
            idle();
        }
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (Math.abs(errorFL) > TOL || Math.abs(errorFR) > TOL || Math.abs(errorBL) > TOL || Math.abs(errorBR) > TOL));
        // all of the motors must reach their tolerance before the robot exits the loop

        // stop the motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }


    // a combination of both the align and pivot function
    // angle has to be small otherwise won't work, this function translates and while maintaining a certain angle
    public void moveMaintainHeading(double x, double y, double pivotAngle, double refAngle, double minSpeed, double maxSpeed, double timeout)
    {
        // run with encoder mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        curTurnAngle = imu.getAngularOrientation().firstAngle - refAngle;
        curTurnAngle = adjustAngles(curTurnAngle);
        errorAngle = pivotAngle - curTurnAngle;

        pivotDst = (int) ((errorAngle / 360.0) * ROBOT_DIAMETER_MM * 3.1415 * COUNTS_PER_MM);

        int xTarget = (int) Math.round(COUNTS_PER_MM * (x * SCALE_OMNI));
        int yTarget = (int) Math.round(COUNTS_PER_MM * (y * SCALE_OMNI));
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
            speedAbsFL = Range.clip(speedAbsFL, minSpeed, maxSpeed);
            speedFL = speedAbsFL * Math.signum(speedFL);  // set sign of speed
            speedFL += pivotSpeed;  // combine movement and pivot speeds

            errorFR = newTargetFR - motorFR.getCurrentPosition();
            speedFR = Kmove * errorFR;
            speedAbsFR = Math.abs(speedFR);
            speedAbsFR = Range.clip(speedAbsFR, minSpeed, maxSpeed);  // clip abs(speed)
            speedFR = speedAbsFR * Math.signum(speedFR);
            speedFR -= pivotSpeed;  // combine movement and pivot speeds

            errorBL = newTargetBL - motorBL.getCurrentPosition();
            speedBL = Kmove * errorBL;
            speedAbsBL = Math.abs(speedBL);
            speedAbsBL = Range.clip(speedAbsBL, minSpeed, maxSpeed);  // clip abs(speed)
            speedBL = speedAbsBL * Math.signum(speedBL);
            speedBL += pivotSpeed;  // combine movement and pivot speeds

            errorBR = newTargetBR - motorBR.getCurrentPosition();
            speedBR = Kmove * errorBR;
            speedAbsBR = Math.abs(speedBR);
            speedAbsBR = Range.clip(speedAbsBR, minSpeed, maxSpeed);
            speedBR = speedAbsBR * Math.signum(speedBR);
            speedBR -= pivotSpeed; // combine movement and pivot speeds

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


    /*

     */
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

        if (isLogging)

        // stop the motorstelemetry.log().add(String.format("avgDistError: %f , EA: %f", avgDistError, errorAngle));
        telemetry.update();
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    // this method drives for seconds, and it can only pivot
    public void moveTimed(double xPower, double yPower, int milliSeconds) throws InterruptedException
    {
        powerFL = xPower + yPower;
        powerFR = -xPower + yPower;
        powerBL = -xPower + yPower;
        powerBR = xPower + yPower;

        // turn on power
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);

        // let it run for x seconds
        sleep(milliSeconds);
        // stop the motors after x seconds
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    // pivot using IMU, but with a reference start angle, but this angle has to be determined (read) before this method is called
    public void pivotWithReference(double targetAngle, double refAngle, double minSpeed, double maxSpeed)
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
            currentAngle = adjustAngles(imu.getAngularOrientation().firstAngle - refAngle);
            errorAngle = adjustAngles(currentAngle - targetAngle);
            pivotSpeed = Math.abs(errorAngle) * Kpivot;
            pivotSpeed = Range.clip(pivotSpeed, minSpeed, maxSpeed); // limit abs speed
            pivotSpeed = pivotSpeed * Math.signum(errorAngle); // set the sign of speed

            // positive angle means CCW rotation
            motorFL.setPower(pivotSpeed);
            motorFR.setPower(-pivotSpeed);
            motorBL.setPower(pivotSpeed);
            motorBR.setPower(-pivotSpeed);

            // allow some time for IMU to catch up
            if (Math.abs(errorAngle) < 5.0)
            {
                sleep(15);
                // stop motors
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                sleep(150);
            }
/*
            sleep(100);
            motorFL.setPower(0.0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
*/
            if (isLogging) telemetry.log().add(String.format("StartAngle: %f, CurAngle: %f, error: %f", refAngle, currentAngle, errorAngle));
            idle();

        } while (opModeIsActive() && (Math.abs(errorAngle) > TOL_ANGLE) );

        // stop motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }


    public void closeGG()
    {
        curGGPos = motorGlyphGrab.getCurrentPosition();
        while (curGGPos > maxGGPos) // CLOSE (counter goes negative when closing)
        {
            curGGPos = motorGlyphGrab.getCurrentPosition();
            errorMaxGG = curGGPos - maxGGPos;
            speedGG = Math.abs(errorMaxGG * KGlyph);
            speedGG = Range.clip(speedGG, 0.09, 0.35);
            speedGG = speedGG * Math.signum(errorMaxGG);
            motorGlyphGrab.setPower(speedGG);
        }
    }

    public void openGG()
    {
        while (curGGPos < minGGPos)
        {
            curGGPos = motorGlyphGrab.getCurrentPosition();
            errorMinGG = curGGPos - minGGPos;
            speedGG = Math.abs(errorMinGG * KGlyph);
            speedGG = Range.clip(speedGG, 0.09, 0.35);
            speedGG = speedGG * Math.signum(errorMinGG);
            motorGlyphGrab.setPower(speedGG);
        }
    }


    public void moveUntilColor(double x, double y, float hue, float saturation)
    {
        // move straight in one direction
        // calculate the power for each motor
        powerFL = x + y;
        powerFR = -x + y;
        powerBL = -x + y;
        powerBR = x + y;

        // while the current hue and saturation is not what the passed in parameters are, keep driving in that direction
        //while (sensorColorRight.red())
        // set power to the motors
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
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