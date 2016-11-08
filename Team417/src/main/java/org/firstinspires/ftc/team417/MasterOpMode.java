package org.firstinspires.ftc.team417;
import android.graphics.Bitmap;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

abstract public class MasterOpMode extends LinearOpMode
{
    DcMotor motorFrontLeft = null;
    DcMotor motorBackLeft = null;
    DcMotor motorFrontRight = null;
    DcMotor motorBackRight = null;
    BNO055IMU imu;
    Orientation angles;

    private ElapsedTime runtime = new ElapsedTime();

    // Declare constants
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    // static final double DRIVE_GEAR_REDUCTION = 0.25;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 6.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    // Vuforia stuff
    public static final String TAG = "Vuforia Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer.CloseableFrame frame;
    Image image = null;
    Image imageRGB565 = null;
    Image imageRGB888 = null;
    int imageFormat;
    Bitmap bm;      // android.graphics
    boolean gamePadButtonA;  // determine which half of beacon to sample

    VuforiaLocalizer vuforia;

    public void initializeHardware()
    {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
    }
}
