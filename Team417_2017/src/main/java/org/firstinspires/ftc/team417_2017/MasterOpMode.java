package org.firstinspires.ftc.team417_2017;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

abstract public class MasterOpMode extends LinearOpMode
{
    // Declare drive motors
    DcMotor motorFL = null;
    DcMotor motorFR = null;
    DcMotor motorBL = null;
    DcMotor motorBR = null;

    // Declare sensors
    BNO055IMU imu; // inertial measurement unit (located within the REV Hub

    // Declare constants
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 6.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_MM = COUNTS_PER_INCH / 25.4; // is 2.34

    public void initializeHardware()
    {
        // Initialize motors to be the hardware motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // get the heading 3 times because first readings aren't accurate (this was tested)
        float angle;
        for (int i = 0; i < 3; i++) {
            sleep(100);
            angle = imu.getAngularOrientation().firstAngle;
        }
    } //-----------------------END OF INITIALIZATION SOFTWARE------------------------


    // normalizing the angle to be between -180 to 180
    public double adjustAngles(double angle)
    {
        while(angle > 180)
            angle -= 360;
        while(angle < -180)
            angle += 360;
        return angle;
    }

    // wait a number of milliseconds
    public void pause(int t) throws InterruptedException
    {
        //we don't use System.currentTimeMillis() because it can be inconsistent
        long initialTime = System.nanoTime();
        while(((System.nanoTime() - initialTime)/1000/1000 < t) && opModeIsActive())
        {
            idle();
        }
    }

    public void omniDrive(double driveAngle, double drivePower, double pivotPower)
    {
        // x is right, or -90 degrees, while y is forwards, which is 0 degrees
        // Calculate drive power (px and py) for both x and y direction
        double px = drivePower * -Math.sin(Math.toRadians(driveAngle));
        double py = drivePower * Math.cos(Math.toRadians(driveAngle));

        // calculate the power for each motor
        double powerFL = px + py + pivotPower;
        double powerFR = px - py + pivotPower;
        double powerBL = px + py + pivotPower;
        double powerBR = -px - py + pivotPower;

        // set power to the motors
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    // Used for calculating distances between 2 points
    double calcDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
