package org.firstinspires.ftc.team8923;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/*
 * This class contains all objects and methods that should be accessible by all OpModes
 */
abstract class Master extends LinearOpMode
{
    // Declares all hardware on robot
    DcMotor motorFL = null;
    DcMotor motorFR = null;
    DcMotor motorBL = null;
    DcMotor motorBR = null;

    // TODO: This just returns 256 for everything, and is likely caused by the wrong sensor type. Fix me
    ColorSensor colorSensor;
    BNO055IMU imu;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    double headingOffset = 0.0;

    // TODO: Confirm these numbers
    // Constants to be used in code. Measurements in millimeters
    static final double GEAR_RATIO = 1.0; // Ratio of driven gear to driving gear
    static final double TICKS_PER_MOTOR_REVOLUTION = 1120.0;
    static final double TICKS_PER_WHEEL_REVOLUTION = TICKS_PER_MOTOR_REVOLUTION / GEAR_RATIO;
    static final double WHEEL_DIAMETER = 4 * 25.4; // 4 inch diameter
    static final double MM_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    static final double MM_PER_TICK = MM_PER_REVOLUTION / TICKS_PER_WHEEL_REVOLUTION;

    // Initialize hardware on robot
    void initHardware()
    {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        // Our drive motors seem to run at this speed
        motorFL.setMaxSpeed(2700);
        motorFR.setMaxSpeed(2700);
        motorBL.setMaxSpeed(2700);
        motorBR.setMaxSpeed(2700);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    // Sends information to Driver Station screen for drivers to see
    void sendTelemetry()
    {
        //TODO: Probably won't need this after testing. It takes up a lot of room, so remove if no longer needed.
        // Drive motor info
        telemetry.addData("FL Enc", formatNumber(motorFL.getCurrentPosition()));
        telemetry.addData("FR Enc", formatNumber(motorFR.getCurrentPosition()));
        telemetry.addData("BL Enc", formatNumber(motorBL.getCurrentPosition()));
        telemetry.addData("BR Enc", formatNumber(motorBR.getCurrentPosition()));

        telemetry.addData("FL", formatNumber(motorFL.getPower()));
        telemetry.addData("FR", formatNumber(motorFR.getPower()));
        telemetry.addData("BL", formatNumber(motorBL.getPower()));
        telemetry.addData("BR", formatNumber(motorBR.getPower()));

        telemetry.update();
    }

    // This allows the robot to drive in any direction and/or turn. Both autonomous and TeleOp use
    // this method, and may need to use some math. 0 degress represents forward
    void driveMecanum(double driveAngle, double drivePower, double turnPower)
    {
        drivePower = Range.clip(drivePower, -1, 1);

        double x = drivePower * -Math.sin(Math.toRadians(driveAngle));
        double y = drivePower * Math.cos(Math.toRadians(driveAngle));

        // Set power for motors. Ratios are correct, but needs scaling (see below)
        double powerFL = y + x - turnPower;
        double powerFR = y - x + turnPower;
        double powerBL = y - x - turnPower;
        double powerBR = y + x + turnPower;

        // Motor powers might be set above 1, so this scales all of the motor powers to stay
        // proportional and within power range
        double scalar = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        // Only apply scalar if greater than 1
        if(scalar < 1)
            scalar = 1;

        // Don't divide by 0
        if(scalar != 0)
        {
            powerFL /= scalar;
            powerFR /= scalar;
            powerBL /= scalar;
            powerBR /= scalar;
        }

        // Set motor powers
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    void stopDriving()
    {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    // Truncates numbers to fit displays better. Not recommended for numbers that span many
    // magnitudes. Also consider the decimal point character.
    private String formatNumber(double number)
    {
        return String.format("%.2f", number);
    }

    // Used for calculating distances between 2 points
    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }
}
