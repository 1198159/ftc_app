package org.firstinspires.ftc.team6220;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;



/*

*/

abstract public class MasterOpMode extends LinearOpMode
{
    //Java Enums cannot act as integer indices as they do in other languages
    //To maintain compatabliity with with a FOR loop, we
    //TODO: find out if java supports an order-independent loop for each member of an array
    //e.g., in python that would be:   for assembly in driveAssemblies:
    //                                    assembly.doStuff()
    public final int FRONT_RIGHT = 0;
    public final int FRONT_LEFT  = 1;
    public final int BACK_LEFT   = 2;
    public final int BACK_RIGHT  = 3;

    DcMotor collectorMotor;
    Servo gateServo;
    Servo collectorServo;
    Servo beaconServo;
    BNO055IMU imu;

    private int EncoderFR = 0;
    private int EncoderFL = 0;
    private int EncoderBL = 0;
    private int EncoderBR = 0;

    double robotXPos = 0;
    double robotYPos = 0;

    //TODO: deal with angles at all starting positions
    double currentAngle = 0.0;

    //used to create global coordinates by adjusting the imu heading based on the robot's starting orientation
    private double headingOffset = 0.0;

    //allows robot to have two possible front ends in teleOp
    boolean leftButtonPusherAsFront = false;



    //used to ensure that the robot drives straight when not attempting to turn
    double targetHeading = 0.0 + headingOffset;

    DriveAssembly[] driveAssemblies;

    DriveSystem drive;

    VuforiaHelper vuforiaHelper;

    ElapsedTime timer = new ElapsedTime();
    double lTime = 0;

    DriverInput driver1;
    DriverInput driver2;
    Launcher launcher = new Launcher(this);

    List<ConcurrentOperation> callback = new ArrayList<>();
    //currently not in use
    /*
    MotorToggler motorToggler;
    MotorToggler motorTogglerReverse;
    */

    public void initializeHardware()
    {
        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);
        //create a driveAssembly array to allow for easy access to motors
        driveAssemblies = new DriveAssembly[4];

        //our robot uses an omni drive, so our motors are positioned at 45 degree angles to motor positions on a normal drive.
        //                                                                             mtr,                          x,    y,  rot,  gear, radius, correction factor
        driveAssemblies[BACK_RIGHT]  = new DriveAssembly(hardwareMap.dcMotor.get("motorBackRight"),  new Transform2D(1.0, 1.0, 135), 1.0, 0.1016, 1.0);
        driveAssemblies[BACK_LEFT]   = new DriveAssembly(hardwareMap.dcMotor.get("motorBackLeft"),   new Transform2D(-1.0,  1.0, 225), 1.0, 0.1016, 1.0);
        driveAssemblies[FRONT_LEFT]  = new DriveAssembly(hardwareMap.dcMotor.get("motorFrontLeft"),  new Transform2D(-1.0, -1.0, 315), 1.0, 0.1016, 1.0);
        driveAssemblies[FRONT_RIGHT] = new DriveAssembly(hardwareMap.dcMotor.get("motorFrontRight"), new Transform2D(1.0,  -1.0,  45), 1.0, 0.1016, 1.0);

        //motors associated with our collection system
        collectorMotor = hardwareMap.dcMotor.get("motorCollector");
        collectorServo = hardwareMap.servo.get("servoCollector");
        gateServo = hardwareMap.servo.get("servoCollectorGate");
        beaconServo = hardwareMap.servo.get("servoBeaconActuator");
        gateServo.setPosition(Constants.GATE_SERVO_DEPLOYED_POSITION);
        collectorServo.setPosition(0.5);
        beaconServo.setPosition(0.0);

        //TODO: Must be disabled if motor encoders are not correctly reporting
        driveAssemblies[FRONT_RIGHT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveAssemblies[FRONT_LEFT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveAssemblies[BACK_LEFT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveAssemblies[BACK_RIGHT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //not currently in use
        /*
        //CodeReview: It seems like your MotorToggler class should handle both of these cases.
        //            You shouldn't have to create two variables (one for forwards, one for backwards).
        //            E.g. your MotorToggler could have setDirection(Direction.Forwards), setDirection(Direction.Backwards)
        //            and then turnOn() would start the motor in that direction.
        motorToggler = new MotorToggler(collectorMotor, 1.0);
        motorTogglerReverse = new MotorToggler(collectorMotor, -1.0);
        */


        //TODO: remove "magic numbers"
        //CodeReview: please don't use magic numbers (0.8). Instead use named constants and
        //            put a comment next to those names explaining where the value comes from (how you derived it)
        //                                          drive assemblies  initial loc:     x    y    w
        drive = new DriveSystem(this, vuforiaHelper, driveAssemblies, new Transform2D(0.0, 0.0, 0.0),
                new PIDFilter[]{
                        new PIDFilter(0.9, 0.0005, -0.1),    //x location control
                        new PIDFilter(0.9, 0.0005, -0.1),    //y location control
                        new PIDFilter(Constants.TURNING_POWER_FACTOR, 0.0000007, -0.001),  //rotation control
                        new PIDFilter(Constants.TURNING_POWER_FACTOR, 0.0, 0.0)}); //rotation control without I value for driving straight

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        //CodeReview: Did we include the calibration json somewhere so it can be found in our program?
        //            If we are going to reference this file, it has to exist, and has to be where the
        //            calibration sample opmode puts it (so it can be found)
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        callback.add(driver1);
        callback.add(driver2);
        callback.add(launcher);
        for(ConcurrentOperation item : callback)
        {
            item.initialize(hardwareMap);
        }
    }

    public void updateCallback(double eTime)
    {
        for(ConcurrentOperation item : callback)
        {
            item.update(eTime);
        }
    }

    //TODO: test encoder function; likely has errors
    //keeps track of the robot's location on the field based on Encoders and IMU; make sure to call once each loop
    public Transform2D updateLocationUsingEncoders(double eTime)
    {

        //x and y positions not considering robot rotation
        double xRawPosition = 0.0;
        double yRawPosition = 0.0;

        //x and y positions taking into account robot rotation
        double xLocation;
        double yLocation;

        //angles relative to starting angle used to determine our x and y positions
        double xAngle;
        double yAngle;

        Transform2D location;

        //angle in degrees for return value
        double currentAngleDegrees = getAngularOrientationWithOffset();

        //converted to radians for Math.sin() function
        currentAngle = currentAngleDegrees * Math.PI / 180;

        EncoderFR = driveAssemblies[FRONT_RIGHT].motor.getCurrentPosition();
        EncoderFL = driveAssemblies[FRONT_LEFT].motor.getCurrentPosition();
        EncoderBL = driveAssemblies[BACK_LEFT].motor.getCurrentPosition();
        EncoderBR = driveAssemblies[BACK_RIGHT].motor.getCurrentPosition();

        //math to calculate x and y positions based on encoder ticks and robot angle
        //factors after EncoderFL are equivalent to circumference / encoder ticks per rotation
        //robotXPos = Math.cos(currentAngle) * (EncoderFL + EncoderFR) * 2 * Math.PI * 0.1016 / (1120 * Math.pow(2, 0.5));
        //robotYPos = Math.sin(currentAngle) * (EncoderFL - EncoderFR) * 2 * Math.PI * 0.1016 / (1120 * Math.pow(2, 0.5));

        //not currently in use
        Transform2D motion = drive.getRobotMotionFromEncoders(eTime);

        //these are shorthand for the encoder derivative for each motor and will be plugged into our encoder function
        double FLencDerivative = driveAssemblies[BACK_RIGHT].getEncoderLinearDerivative(eTime);
        double FRencDerivative = driveAssemblies[BACK_RIGHT].getEncoderLinearDerivative(eTime);
        double BLencDerivative = driveAssemblies[BACK_RIGHT].getEncoderLinearDerivative(eTime);
        double BRencDerivative = driveAssemblies[BACK_RIGHT].getEncoderLinearDerivative(eTime);


        yAngle = Math.PI / 2 + currentAngle;
        xAngle = Math.PI / 2 - yAngle;

        //math to calculate x and y positions based on encoder ticks and not accounting for angle
        xRawPosition += eTime * ((FLencDerivative + FRencDerivative -(BLencDerivative + BRencDerivative)) / (4 * Math.sqrt(2)));
        yRawPosition += eTime * ((FLencDerivative + BLencDerivative -(FRencDerivative + BRencDerivative)) / (4 * Math.sqrt(2)));

        //final location utilizing angles
        xLocation = yRawPosition * Math.cos(yAngle) + xRawPosition * Math.cos(xAngle);
        yLocation = yRawPosition * Math.sin(yAngle) - xRawPosition * Math.sin(xAngle);

        location = new Transform2D(xLocation, yLocation, currentAngleDegrees);

        telemetry.addData("X:", xLocation);
        telemetry.addData("Y:", yLocation);
        telemetry.addData("W:", currentAngleDegrees);
        telemetry.update();


        //CodeReview: do you need this return value? (does any caller need it?)
        //            Seems like all that's needed is to update location in this method.
        return location;
    }

    //uses solely encoders to move the robot to a desired location
    public void navigateUsingEncoders(double targetX, double targetY, ElapsedTime timer)
    {
        double positionOffsetMagnitude = Math.sqrt(Math.pow(targetX - drive.robotLocation.x,2)+Math.pow(targetY - drive.robotLocation.y,2));
        while ((positionOffsetMagnitude > Constants.POSITION_TOLERANCE) && opModeIsActive())
        {
            positionOffsetMagnitude = Math.sqrt(Math.pow(targetX - drive.robotLocation.x,2)+Math.pow(targetY - drive.robotLocation.y,2));
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            drive.navigateTo(targetX, targetY);

            drive.robotLocation = updateLocationUsingEncoders(eTime);

            idle();
        }
    }

    //other opmodes must go through this method to prevent others from blithely changing headingOffset
    public void setRobotStartingOrientation(double newValue)
    {
        headingOffset = newValue;
    }

    //prevents angle differences from being out of range
    public double normalizeRotationTarget(double finalAngle, double initialAngle)
    {
        double diff = finalAngle - initialAngle;

        while (Math.abs(diff) > 180)
        {
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
    public double getAngularOrientationWithOffset()
    {
        double correctedHeading = normalizeAngle(imu.getAngularOrientation().firstAngle + headingOffset);

        return correctedHeading;
    }

    //uses vuforia instead of imu
    public double getRobotAngleUsingVuforia()
    {
        vuforiaHelper.updateLocation();
        return Orientation.getOrientation(vuforiaHelper.lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle + headingOffset;
    }

    //wait a number of milliseconds
    public void pause(int t) throws InterruptedException
    {
        //we don't use System.currentTimeMillis() because it can be inconsistent
        long initialTime = System.nanoTime();
        while((System.nanoTime() - initialTime)/1000/1000 < t)
        {
            idle();
        }
    }

    public void stopAllDriveMotors()
    {
        driveAssemblies[FRONT_RIGHT].setPower(0.0);
        driveAssemblies[FRONT_LEFT].setPower(0.0);
        driveAssemblies[BACK_LEFT].setPower(0.0);
        driveAssemblies[BACK_RIGHT].setPower(0.0);
    }
}
