package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Main robot code center, hold all the information needed to run the robot
 */

public abstract class Master extends LinearOpMode
{

    // Declare motors here
    DcMotor motorFL = null;
    DcMotor motorFR = null;
    DcMotor motorBL = null;
    DcMotor motorBR = null;
    DcMotor motorGG = null;


    double slowModeDivisor;

    // Declare servos here
    Servo servoJJ = null;
    Servo servoGGL = null;
    Servo servoGGR = null;

    // Declare any neccessary sensors here
    BNO055IMU imu;

    // Declare any robot-wide variables here
    double SlowModeDivisor = 1.0;

    // Declare constants here
    private static  final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_MOTOR_REVOLUTION = 1120.0; // Standard number for Andymark Neverest 40's
    private static final double TICKS_PER_WHEEL_REVOLUTION = TICKS_PER_MOTOR_REVOLUTION / GEAR_RATIO;
    private static final double WHEEL_DIAMETER = 4 * 25.4; // 4 inch diameter
    private static final double MM_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    //private static final double CORRECTION_FACTOR = 0.92;
    static final double MM_PER_TICK = MM_PER_REVOLUTION / TICKS_PER_WHEEL_REVOLUTION/* * CORRECTION_FACTOR*/;
    //Servos constants
    double SERVO_JJ_UP = 0.8; //Port 5, Hub 1
    double SERVO_JJ_DOWN = 0.1;

    //declare IMU
    double currentRobotAngle;
    double jX;
    double jY;
    double kAngle;


    void InitHardware()
    {
        // Motors here
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        // Servos here
        servoJJ = hardwareMap.get(Servo.class, "servoJJ");

        servoJJ.setPosition(SERVO_JJ_UP);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Servos here

        // Sensors here
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    // 45 denotes the angle at which the motors are mounted in referece to the chassis frame
    void driveOmni45(double driveAngle, double drivePower, double turnPower)
    {
        // Calculate out x and y directions of drive power, where y is forward (0 degrees) and x is right (-90 degrees)
        double x = drivePower * -Math.sin(Math.toRadians(driveAngle));
        double y = drivePower * Math.cos(Math.toRadians(driveAngle));

        /*
         * to explain:
         * each wheel omni wheel exerts a force in one direction like tank but differs in the fact that they have rollers mounted perpendicular
         * to the wheels outer edge so they can passively roll at 90 degrees to the wheel's facing. This means that with the wheels mounted at 45
         * degrees to the chassis frame and assuming the left hand rule for the motors, each wheel's power needs to be 90 degrees out of phase
         * from the previous wheel on the unit circle starting with positive y and x for FL and going clockwise around the unit circle and robot
         * from there
         */

        double powerFL = y + x + turnPower;
        double powerFR = -y + x + turnPower;
        double powerBL = y - x + turnPower;
        double powerBR = -y - x + turnPower;

        double scalar = Math.max(Math.abs(powerFL),  Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        if(scalar < 1)
            scalar = 1;

        powerFL /= (scalar * SlowModeDivisor);
        powerFR /= (scalar * SlowModeDivisor);
        powerBL /= (scalar * SlowModeDivisor);
        powerBR /= (scalar * SlowModeDivisor);

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }



    // normalizing the angle to be between -180 to 180
    public double adjustAngles(double angle)
    {
        while(angle > 180)
            angle -= 360;
        while(angle < -180)
            angle += 360;
        return angle;
    }

    void stopDriving()
    {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    // Used for calculating distances between 2 points
    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    boolean buttonsAreReleased(Gamepad pad)
    {
        return !(pad.a || pad.b || pad.x || pad.y || pad.left_bumper || pad.right_bumper
                || pad.dpad_up || pad.dpad_down || pad.dpad_left || pad.dpad_right
                || pad.left_stick_button || pad.right_stick_button
                || pad.start || pad.back || pad.guide || pad.left_trigger > 0.35
                || pad.right_trigger > 0.35);
    }
}
