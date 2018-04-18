package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;

import java.util.ArrayList;
import java.util.List;

abstract public class MasterOpMode extends LinearOpMode
{
    // Uses the robot's starting orientation to modify imu output and create global heading
    private double headingOffset = 0;

    /*
     Polynomial for adjusting input from joysticks to allow for ease of driving.  A polynomial that
     is concave up in the 1st quadrant is preferable to a 1st degree polynomial, since a driver
     generally needs more control in the low speed range than the high range.
    */
    //                                             y = 0 + 0.25x + 0 + 0.75x^3
    Polynomial stickCurve = new Polynomial(new double[]{ 0, 0.25, 0, 0.75});

    // Contains useful vuforia methods
    VuforiaHelper vuforiaHelper;

    ElapsedTime timer = new ElapsedTime();
    double lTime = 0;

    DriverInput driver1;
    DriverInput driver2;

    // Classes that encapsulate distinct hardware systems
    ArmMechanism armMechanism;
    GlyphMechanism glyphMechanism;

    ColorSensor sensorRGB;


    // Stores encoder values for each of the 4 heights that glyphs can be scored at.
    int[] glyphHeights = new int[4];

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // Values is a reference to the hsvValues array.
    final float values[] = hsvValues;


    // Declare filters.  We have PID for turning, PID for encoder navigation, PID for moving the----
    // glyphter, and 3 acceleration-limiting filters (in autonomous and teleOp).
    PIDFilter rotationFilter;
    PIDFilter translationFilter;
    PIDFilter glyphterFilter;

    AccelerationFilter navigationAccelFilter;
    AccelerationFilter driveAccelFilter;
    AccelerationFilter turnAccelFilter;
    //----------------------------------------------------------------------------------------------

    // Declare hardware devices---------------------------------------
    BNO055IMU imu;

     // Motors----------------------
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    DcMotor motorGlyphter;
      // Motor orientations are from top view of robot with glyph mechanism in front
    CRServo motorCollectorLeft;
    CRServo motorCollectorRight;

    DcMotor motorArm;
     //-----------------------------

     // Servos----------------------
    Servo lateralJewelServo;
    Servo verticalJewelServo;

    Servo glyphClipServo;

    Servo wristServo;
    Servo grabberServo;

    CRServo collectorTurntableServo;
     //------------------------------
    //-----------------------------------------------------------------

    // Servo togglers
    ServoToggler verticalJewelServoToggler;
    ServoToggler grabberServoToggler;
    ServoToggler wristServoToggler;
    ServoToggler glyphClipServoToggler;

    // Booleans that allow us to choose what parts of the robot we are and aren't using in each OpMode
    public boolean isDriveTrainAttached = true;
    public boolean isArmAttached = true;
    public boolean isGlyphMechAttached = true;
    public boolean isJewelJostlerAttached = true;

    // Create a list of tasks to accomplish in order
    List<ConcurrentOperation> callback = new ArrayList<>();



    public void initializeRobot()
    {
        // Initialize encoder values for scoring glyphs at different heights.  These are passed into
        // glyphMechanism and used for the 2nd driver controls
        glyphHeights[0] = Constants.HEIGHT_1;
        glyphHeights[1] = Constants.HEIGHT_2;
        glyphHeights[2] = Constants.HEIGHT_3;
        glyphHeights[3] = Constants.HEIGHT_4;

        // Initialize robot mechanism classes----------------------------
        armMechanism = new ArmMechanism(this);
        glyphMechanism = new GlyphMechanism(this, glyphHeights);
        //---------------------------------------------------------------

        // Instantiated gamepad classes that must be updated each callback
        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        // Add necessary items to callback---------------
        callback.add(driver1);
        callback.add(driver2);
        callback.add(glyphMechanism);
        //-----------------------------------------------


         // Check to see what parts of the robot are attached.  Some programs (e.g., autonomous and -------------------------
         // tests) may want to ignore parts of the robot that don't need to be used
        if (isDriveTrainAttached)
        {
            // Drive motors
            motorFL = hardwareMap.dcMotor.get("motorFrontLeft");
            motorFR = hardwareMap.dcMotor.get("motorFrontRight");
            motorBL = hardwareMap.dcMotor.get("motorBackLeft");
            motorBR = hardwareMap.dcMotor.get("motorBackRight");

            // Set motor attributes and behaviors------------------------------
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

            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorFR.setPower(0);
            //-------------------------------------------------------------------
        }
        if (isGlyphMechAttached)
        {
            // Initialize glyph mechanism devices--------------------------------
            collectorTurntableServo = hardwareMap.crservo.get("collectorTurntableServo");

            motorGlyphter = hardwareMap.dcMotor.get("motorGlyphter");
            motorCollectorLeft = hardwareMap.crservo.get("motorLeftCollector");
            motorCollectorRight = hardwareMap.crservo.get("motorRightCollector");
            //-------------------------------------------------------------------

            // Set device attributes and behaviors--------------------------------
            motorGlyphter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorGlyphter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorGlyphter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //motorCollectorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //motorCollectorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //motorCollectorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorCollectorLeft.setPower(0);
            motorCollectorRight.setPower(0);


            sensorRGB = hardwareMap.colorSensor.get("sensor_color");
            collectorTurntableServo.setPower(-Constants.MINIMUM_TURNTABLE_POWER);
            //--------------------------------------------------------------------
        }
        if (isArmAttached)
        {
            // Initialize arm devices-------------------------------------
            motorArm = hardwareMap.dcMotor.get("motorArm");

            wristServo = hardwareMap.servo.get("wristServo");
            grabberServo = hardwareMap.servo.get("grabberServo");
            //------------------------------------------------------------

            // Servo togglers
            grabberServoToggler = new ServoToggler(grabberServo, Constants.GRABBER_SERVO_RELEASE, Constants.GRABBER_SERVO_GRIP);
            wristServoToggler = new ServoToggler(wristServo, Constants.WRIST_SERVO_RETRACTED, Constants.WRIST_SERVO_DEPLOYED);

            // Set device attributes and behaviors------------------------
            motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motorArm.setPower(0);
            wristServo.setPosition(Constants.WRIST_SERVO_INIT);
            grabberServoToggler.deploy();
            //------------------------------------------------------------
        }
        if (isJewelJostlerAttached)
        {
            // Jewel servos
            verticalJewelServo = hardwareMap.servo.get("verticalJewelServo");
            lateralJewelServo = hardwareMap.servo.get("lateralJewelServo");

            // Servo togglers
            verticalJewelServoToggler = new ServoToggler(verticalJewelServo, Constants.VERTICAL_JEWEL_SERVO_RETRACTED, Constants.VERTICAL_JEWEL_SERVO_DEPLOYED);

            // Set initial servo positions
            verticalJewelServo.setPosition(Constants.VERTICAL_JEWEL_SERVO_INIT);
            lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_INIT);
        }
        //--------------------------------------------------------------------------------------------------------------


        // Create and initialize glyph clip.  We use this to score our first glyph in autonomous.
        glyphClipServo = hardwareMap.servo.get("glyphClipServo");
        glyphClipServoToggler = new ServoToggler(glyphClipServo, Constants.GLYPH_CLIP_SERVO_RETRACTED, Constants.GLYPH_CLIP_SERVO_DEPLOYED);
        //glyphClipServoToggler.retract();


        // todo REV imu can occasionally taking a long time to initialize or even fail to do so; why is this?
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu". Certain parameters must be specified before using the imu.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Initialize PID filters
        rotationFilter = new PIDFilter(Constants.ROTATION_P, Constants.ROTATION_I, Constants.ROTATION_D);
        translationFilter = new PIDFilter(Constants.TRANSLATION_P, Constants.TRANSLATION_I, Constants.TRANSLATION_D);
        glyphterFilter = new PIDFilter(Constants.GLYPHTER_P, Constants.GLYPHTER_I, Constants.GLYPHTER_D);

        // Inititialize acceleration filters
        navigationAccelFilter = new AccelerationFilter(this, Constants.NAV_ACCEL, Constants.NAV_DECEL);
        driveAccelFilter = new AccelerationFilter(this, Constants.DRIVE_ACCEL, Constants.DRIVE_DECEL);
        turnAccelFilter = new AccelerationFilter(this, Constants.TURN_ACCEL, Constants.TURN_DECEL);

        //todo Initialize all separate hardware systems here
        for (ConcurrentOperation item : callback)
        {
            item.initialize(hardwareMap);
        }
    }


    /*
     General method for driving robot.  Input is given as a vector in polar form with
     (r, theta) = (drivePower, driveAngle)

     Table for mecanum drive motor directions (clockwise = positive):

                  FL      FR      BL       BR
     rotate -w    -        -      -        -
     rotate +w    +        +      +        +
     forward      -        +      -        +
     backward     +        -      +        -
     right        -        -      +        +
     left         +        +      -        -
     diag. left   +        0      0        -
     diag. right  -        0      0        +
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
        double y = drivePower * Math.sin(Math.toRadians(driveAngle));
        double x = drivePower * Math.cos(Math.toRadians(driveAngle));

        // Signs for x, y, and w are based on the motor configuration and inherent properties of mecanum drive
        double powerFL = -x - y + w;
        double powerFR = -x + y + w;
        double powerBL = x - y + w;
        double powerBR = x + y + w;

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
        // Telemetry for debugging motor power inputs
        /*
        telemetry.addData("translation power: ", x);
        telemetry.addData("vertical power: ", y);
        telemetry.addData("rotational power: ", w);
        */
        telemetry.update();
    }


    // Other opmodes must go through this method to prevent others from unnecessarily changing headingOffset
    void setRobotStartingOrientation(double newValue)
    {
        headingOffset = newValue;
    }


    // Prevents angle differences from being outside the range -180 to 180 degrees
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


    // Updates every item with elapsed time at the end of the main loop; ensures that operations
    // based on a timer are executed on time
    public void updateCallback(double eTime)
    {
        for (ConcurrentOperation item : callback)
        {
            item.update(eTime);
        }
    }


    // Note:  time parameter is in seconds
    // Waits for a specified time while giving each hardware system the ability to function simultaneously
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


    void stopDriveMotors()
    {
        if(!isDriveTrainAttached)
        {
            telemetry.addLine("Drive is not attached!");
            telemetry.update();
            return;
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
}
