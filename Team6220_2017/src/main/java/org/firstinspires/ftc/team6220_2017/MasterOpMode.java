package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

abstract public class MasterOpMode extends LinearOpMode
{
    // Used to create global coordinates by adjusting the imu heading based on the robot's starting orientation
    private double headingOffset = 0.0;

    // Polynomial for adjusting input from joysticks to allow for ease of driving
    //                                            y = 0.0 + (1/4)x + 0.0 + (3/4)x^3
    Polynomial stickCurve = new Polynomial(new double[]{ 0.0, 0.25, 0.0, 0.75 });

    // Note: not used
    // Used to ensure that the robot drives straight when not attempting to turn
    double targetHeading = 0.0 + headingOffset;

    // Contains useful vuforia methods
    VuforiaHelper vuforiaHelper;

    ElapsedTime timer = new ElapsedTime();
    double lTime = 0;

    DriverInput driver1;
    DriverInput driver2;

    // todo How to deal with this class being abstract?
    // Class that is used to run the relic arm
    ArmMechanism armMechanism;

    PIDFilter RotationFilter;
    PIDFilter TranslationFilter;

    // Declare hardware devices--------------------
    BNO055IMU imu;

    // Motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    DcMotor motorArm;
    //

    // Servos
    Servo lateralJewelServo;
    Servo verticalJewelServo;

    Servo wristServo;
    Servo jointServo;
    //
    //-----------------------------------------------

    // Servo togglers
    ServoToggler verticalJewelServoToggler;
    //

    // Booleans that allow us to choose what parts of the robot we are using in each OpMode
    public boolean isDriveTrainAttached = true;
    public boolean isArmAttached = true;

    //create a list of tasks to accomplish in order
    List<ConcurrentOperation> callback = new ArrayList<>();

    public void initialize()
    {
        // Instantiated classes that must be updated each loop in callback
        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        callback.add(driver1);
        callback.add(driver2);
        //

        // Initialize hardware devices--------------------------
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Jewel servos
        verticalJewelServo = hardwareMap.servo.get("servoVerticalJewel");
        lateralJewelServo = hardwareMap.servo.get("servoLateralJewel");
        //

        // Servo togglers
        verticalJewelServoToggler = new ServoToggler(verticalJewelServo, Constants.VERTICAL_JEWEL_SERVO_RETRACTED, Constants.VERTICAL_JEWEL_SERVO_DEPLOYED);
        //

        // Set initial servo positions
        verticalJewelServoToggler.setToStartingPosition();
        lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_NEUTRAL);
        //

        if(isDriveTrainAttached)
        {
            // Drive motors
            motorFL = hardwareMap.dcMotor.get("motorFrontLeft");
            motorFR = hardwareMap.dcMotor.get("motorFrontRight");
            motorBL = hardwareMap.dcMotor.get("motorBackLeft");
            motorBR = hardwareMap.dcMotor.get("motorBackRight");
            //

            //todo Make sure to create variables to store encoder values for autonomous method driveToPosition
            // Set motor attributes and behaviors--------------------------
            motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorFL.setPower(0.0);
            motorFR.setPower(0.0);
            motorBL.setPower(0.0);
            motorFR.setPower(0.0);
            //------------------------------------------------------------
        }

        if(isArmAttached)
        {
            // Arm devices-------------------------------------
            motorArm = hardwareMap.dcMotor.get("motorArm");

            wristServo = hardwareMap.servo.get("servoWrist");
            jointServo = hardwareMap.servo.get("servoJoint");
            //-------------------------------------------------

            // Set motor attributes and behaviors--------------
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorArm.setPower(0.0);
            //-------------------------------------------------
        }
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
        //------------------------------------------------

        //todo Adjust these experimentally
        RotationFilter = new PIDFilter(0.7, 0.0, 0.0);
        TranslationFilter = new PIDFilter(1.0, 0.0, 0.2);

        //todo Change servo arm to separate class with objects initialized here
        for (ConcurrentOperation item : callback)
        {
            item.initialize(hardwareMap);
        }
    }

    /*
     General method for driving robot
     Input is given as a vector in polar form with (r,theta)=(drivePower,driveAngle)

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
        if(!isDriveTrainAttached)
        {
            telemetry.addLine("Drive is not attached!");
            telemetry.update();
            return;
        }

        // Convert drive angle and power to x and y components
        double y = drivePower * Math.sin(driveAngle);
        double x = drivePower * Math.cos(driveAngle);

        // Signs for x, y, and w are based on the motor configuration and inherent properties of mecanum drive
        double powerFL = -x - y - w;
        double powerFR = -x + y - w;
        double powerBL = x - y - w;
        double powerBR = x + y - w;

        // Scale powers-------------------------
        /*
         Motor powers might be set above 1 (e.g., x + y = 1 and w = -0.8), so we must scale all of
         the powers to ensure they are proportional and within the range {-1.0, 1.0}
        */
        double powScalar = Math.max(Math.abs(powerFL),
                           Math.max(Math.abs(powerFR),
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
        //--------------------------------------

        // Power motors with corrected inputs
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);

        // todo How to make this telemetry not interfere with that of other classes?
        /*
        // Telemetry for debugging motor power inputs
        telemetry.addData("translation power: ", x);
        telemetry.addData("vertical power: ", y);
        telemetry.addData("rotational power: ", w);
        telemetry.update();
        */
    }

    // Other opmodes must go through this method to prevent others from unnecessarily changing headingOffset
    void setRobotStartingOrientation(double newValue) {
        headingOffset = newValue;
    }

    // Prevents angle differences from being out of range
    public double normalizeRotationTarget(double finalAngle, double initialAngle)
    {
        double diff = finalAngle - initialAngle;

        while (Math.abs(diff) > 180)
        {
            diff -= Math.signum(diff) * 360;
        }

        return diff;
    }

    // Prevents a single angle from being outside the range -180 to 180 degrees
    public double normalizeAngle(double rawAngle)
    {
        while (Math.abs(rawAngle) > 180)
        {
            rawAngle -= Math.signum(rawAngle) * 360;
        }

        return rawAngle;
    }

    // Takes into account headingOffset to utilize global orientation
    double getAngularOrientationWithOffset()
    {
        double correctedHeading = normalizeAngle(imu.getAngularOrientation().firstAngle + headingOffset);

        return correctedHeading;
    }

    // Finds distance between 2 points
    double calculateDistance(double dx, double dy)
    {
        double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        return distance;
    }

    /*
     Updates every item with elapsed time at the end of the main loop; ensures that operations
     based on a timer are executed on time
    */
    public void updateCallback(double eTime)
    {
        for (ConcurrentOperation item : callback)
        {
            item.update(eTime);
        }
    }

    // Waits a number of milliseconds
    void pause(int t) throws InterruptedException
    {
        //we don't use System.currentTimeMillis() because it can be inconsistent
        long initialTime = System.nanoTime();
        while((System.nanoTime() - initialTime)/1000/1000 < t && opModeIsActive())
        {
            idle();
        }
    }

    void stopAllDriveMotors()
    {
        if(!isDriveTrainAttached)
        {
            telemetry.addLine("Drive is not attached!");
            telemetry.update();
            return;
        }

        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }
}
