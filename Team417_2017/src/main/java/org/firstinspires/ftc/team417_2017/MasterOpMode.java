package org.firstinspires.ftc.team417_2017;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

abstract public class MasterOpMode extends LinearOpMode
{
    // Declare drive motors
    DcMotor motorFL = null; // port 0
    DcMotor motorFR = null; // port 3
    DcMotor motorBL = null; // port 1
    DcMotor motorBR = null; // port 2

    // Declare Glyph manipulator motors
    DcMotor motorGlyphLeft = null; // port 0
    DcMotor motorGlyphRight = null; // port 1
    DcMotor motorGlyphGrab = null; // port 2

    // Declare servo, jewel servo
    Servo servoJewel = null; // port 1

    // Declare sensors
    BNO055IMU imu; // inertial measurement unit (located within the REV Hub)
    //ColorSensor sensorColorLeft; // port 1
    //ColorSensor sensorColorRight; // port 2

    // Declare constants
    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 6.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_MM = COUNTS_PER_INCH / 25.4; // is 2.34

    // Servo init and low positions
    static final double JEWEL_INIT = 0.965;
    static final double JEWEL_LOW = 0.38;

    // declare color sensor variables
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};
    float hsvLeft[] = {0F,0F,0F};
    float hsvRight[] = {0F,0F,0F};

    // declare motor powers
    double powerFL;
    double powerFR;
    double powerBL;
    double powerBR;

    double powerGlyphUp = 0.5;
    double powerGlyphDown = -0.5;
    double powerGlyphGrab = 0.2;

    double px;
    double py;

    public void initializeHardware()
    {
        // Initialize motors to be the hardware motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorGlyphLeft = hardwareMap.dcMotor.get("motorGlyphUp");
        motorGlyphRight = hardwareMap.dcMotor.get("motorGlyphDown");
        motorGlyphGrab = hardwareMap.dcMotor.get("motorGlyphGrab");

        // get a reference to the color sensor.
        //sensorColorLeft = hardwareMap.get(ColorSensor.class, "sensorColorLeft");
        //sensorColorRight = hardwareMap.get(ColorSensor.class, "sensorColorRight");

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorGlyphLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlyphRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorGlyphGrab.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

/*
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorGlyphLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorGlyphRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorGlyphGrab.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reverse front and back right motors just for TeleOp
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        motorGlyphLeft.setDirection(DcMotor.Direction.FORWARD);
        motorGlyphRight.setDirection(DcMotor.Direction.REVERSE);
        motorGlyphGrab.setDirection(DcMotor.Direction.FORWARD);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        motorGlyphLeft.setPower(0);
        motorGlyphRight.setPower(0);
        motorGlyphGrab.setPower(0);

        // Initialize servos
        servoJewel = hardwareMap.servo.get("servoJewel");

        servoJewel.setPosition(JEWEL_INIT);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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

    public float DetermineLineHue(ColorSensor sensorColor)
    {
        // Convert the RGB values to HSV values
        Color.RGBToHSV((sensorColor.red() * 255) / 800, (sensorColor.green() * 255) / 800, (sensorColor.blue() * 255) / 800, hsvValues);
        return hsvValues[0]; // return the hue (index 0) of HSV
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