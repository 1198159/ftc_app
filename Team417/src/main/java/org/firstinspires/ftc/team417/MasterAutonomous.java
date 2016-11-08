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
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * Program used to control Drive-A-Bots.
 * This can be a good reference for drive controls.
 */
@Autonomous(name="Master Autonomous", group = "Swerve")
// @Disabled

public class MasterAutonomous extends LinearOpMode
{
    DcMotor motorFrontLeft = null;
    DcMotor motorFrontRight = null;
    DcMotor motorBackLeft = null;
    DcMotor motorBackRight = null;

    BNO055IMU imu;
    Orientation angles;

    // HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    //static final double DRIVE_GEAR_REDUCTION = 0.25;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 6.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException

    {
        // Initialize hardware and other important things
        initializeRobot();
        telemetry.addData("Path", "InitDone");
        telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        forwards(-39, 3, 0.3);
        sleep(1000);
        pivot(60, 0.3);
        sleep(500);
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
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        float angle;
        for (int i = 0; i < 3; i++) {
            sleep(100);
            angle = imu.getAngularOrientation().firstAngle;
        }
        sleep(200); // give it enough time before you press run
        // Initialize motors to be the hardware motors
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // run to position mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
       motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

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
                (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorBackLeft.isBusy() && motorBackRight.isBusy()));

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
            pivotSpeed = Range.clip(pivotSpeed, 0.18, 0.8); // limit abs speed
            pivotSpeed = - pivotSpeed * Math.signum(error); // set the sign of speed


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

        } while (opModeIsActive() && (Math.abs(error) > 0.3 || Math.abs(errorP1) > 0.3 || Math.abs(errorP2) > 0.3) );

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
            incDst = Range.clip(incDst, 0.15, 5); // limit incDst
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



    public double adjustAngles(double angle)
    {
        while(angle > 180)
            angle -= 360;
        while(angle < -180)
            angle += 360;
        return angle;
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