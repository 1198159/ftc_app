package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;

abstract public class MasterOpMode extends LinearOpMode {
    //TODO: deal with angles at all starting positions
    double currentAngle = 0.0;

    //used to create global coordinates by adjusting the imu heading based on the robot's starting orientation
    private double headingOffset = 0.0;

    //used to ensure that the robot drives straight when not attempting to turn
    double targetHeading = 0.0 + headingOffset;

    //contains useful vuforia functions
    VuforiaHelper vuforiaHelper;

    ElapsedTime timer = new ElapsedTime();
    double lTime = 0;

    DriverInput driver1;
    DriverInput driver2;

    PIDFilter RotationControlFilter;

    //declare hardware devices
    BNO055IMU imu;

    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorArm;

    //servo that operates the jewel arm
    Servo golfClubServo;
    Servo hingeServo;
    //

    //create a list of tasks to accomplish in order
    List<ConcurrentOperation> callback = new ArrayList<>();

    public void initializeHardware()
    {
        // instantiated classes that must be updated each loop to callback
        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        callback.add(driver1);
        callback.add(driver2);
        //

        // initialize hardware devices
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        golfClubServo = hardwareMap.servo.get("servoGolfClub");
        hingeServo = hardwareMap.servo.get("servoHinge");
        //

        // set modes and initial positions
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);

        golfClubServo.setPosition(0.5);
        hingeServo.setPosition(0.5);
        //

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu". Certain parameters must be specified before using the imu.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        //todo adjust for robot
        RotationControlFilter = new PIDFilter(0.5, 0.0, 0.0);

        //todo What is the purpose of this?
        for (ConcurrentOperation item : callback)
        {
            item.initialize(hardwareMap);
        }
    }

    /*
    general method for driving robot
    driveAngle = 0 when driving along robot's y-axis (like compass heading); positive angle values are
    counterclockwise from y-axis (like math heading)

    Table for mecanum drive motor directions (counterclockwise = positive):

                 FL      FR      BL      BR
    rotate -w    +        -      +        -
    rotate +w    -        +      -        +
    forward      +        +      +        +
    backward     -        -      -        -
    left         -        +      +        -
    right        +        -      -        +
    diag. left   0        +      +        0
    diag. right  +        0      0        +
    */
    void driveMecanum(double driveAngle, double drivePower, double w)
    {
        //convert drive angle to x and y components from gamepad input vector (drive angle and drive power);
        //x and y are switched
        double y = -drivePower * Math.sin(driveAngle);
        double x = drivePower * Math.cos(driveAngle);

        //signs for x, y, and w are based on inherent properties of mecanum drive and the gamepad
        double powerFL = -x + y - w;
        double powerFR = -x - y - w;
        double powerBL = x + y - w;
        double powerBR = x - y - w;

        //-------------------------
        /*
         Motor powers might be set above 1 (e.g., x + y = 1 and w = -0.8), so we must scale all of
         the powers to ensure they are proportional and within the range {-1.0, 1.0}
        */
        double powScalar = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));
        /*
         However, powScalar should only be applied if it is greater than 1. Otherwise, we could
         unintentionally increase powers or even divide by 0
        */
        if(powScalar < 1)
            powScalar = 1;

        powerFL /= powScalar;
        powerFR /= powScalar;
        powerBL /= powScalar;
        powerBR /= powScalar;
        //-------------------------

        //power motors with corrected inputs
        motorFrontLeft.setPower(powerFL);
        motorFrontRight.setPower(powerFR);
        motorBackLeft.setPower(powerBL);
        motorBackRight.setPower(powerBR);

        //telemetry for debugging motor power inputs
        telemetry.addData("translation power: ", x);
        telemetry.addData("vertical power: ", y);
        telemetry.addData("rotational power: ", w);
        telemetry.update();
    }

    //updates every item with elapsed time at the end of the main loop; ensures that operations
    //based on a timer are executed on time
    public void updateCallback(double eTime)
    {
        for (ConcurrentOperation item : callback)
        {
            item.update(eTime);
        }
    }

    //other opmodes must go through this method to prevent others from unnecessarily changing headingOffset
    void setRobotStartingOrientation(double newValue) {
        headingOffset = newValue;
    }

    //prevents angle differences from being out of range
    public double normalizeRotationTarget(double finalAngle, double initialAngle)
    {
        double diff = finalAngle - initialAngle;

        while (Math.abs(diff) > 180) {
            diff -= Math.signum(diff) * 360;
        }

        return diff;
    }

    //prevents a single angle from being outside the range -180 to 180 degrees
    public double normalizeAngle(double rawAngle)
    {
        while (Math.abs(rawAngle) > 180)
        {
            rawAngle -= Math.signum(rawAngle) * 360;
        }

        return rawAngle;
    }

    //takes into account headingOffset to utilize global orientation
    double getAngularOrientationWithOffset()
    {
        double correctedHeading = normalizeAngle(imu.getAngularOrientation().firstAngle + headingOffset);

        return correctedHeading;
    }

    //finds distance between 2 points
    double calculateDistance(double dx, double dy)
    {
        double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        return distance;
    }

    //wait a number of milliseconds
    void pause(int t) throws InterruptedException
    {
        //we don't use System.currentTimeMillis() because it can be inconsistent
        long initialTime = System.nanoTime();
        while((System.nanoTime() - initialTime)/1000/1000 < t)
        {
            idle();
        }
    }

    void stopAllDriveMotors()
    {
        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
    }
}
