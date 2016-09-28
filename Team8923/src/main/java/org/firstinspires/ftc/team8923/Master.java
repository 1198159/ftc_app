package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class contains all objects and methods that should be accessible by all OpModes
 */
abstract class Master extends LinearOpMode
{
    // Declares all hardware on robot
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;

    // Initialize hardware on robot
    void initHardware()
    {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        // TODO: Find out actual value for these
        //motorFL.setMaxSpeed(168000);
        //motorFR.setMaxSpeed(168000);
        //motorBL.setMaxSpeed(168000);
        //motorBR.setMaxSpeed(168000);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Sends information to Driver Station screen for drivers to see
    void sendTelemetry()
    {
        //TODO: Probably won't need this after testing. It takes up a lot of room, so remove if no longer needed.
        // Drive motor info
        telemetry.addData("FL", truncateNumber(motorFL.getPower(), 4));
        telemetry.addData("FR", truncateNumber(motorFR.getPower(), 4));
        telemetry.addData("BL", truncateNumber(motorBL.getPower(), 4));
        telemetry.addData("BR", truncateNumber(motorBR.getPower(), 4));

        telemetry.update();
    }

    // This allows the robot to drive in any direction and/or turn. Both autonomous and TeleOp use
    // this method, and may need to use some math
    void driveMecanum(double driveAngle, double drivePower, double turnPower)
    {
        double x = drivePower * Math.cos(driveAngle);
        double y = drivePower * Math.sin(driveAngle);

        // Set power for motors. Ratios are correct, but needs scaling (see below)
        double powerFL = y - x + turnPower;
        double powerFR = y + x - turnPower;
        double powerBL = y + x + turnPower;
        double powerBR = y - x - turnPower;

        // Motor powers might be set above 1, so this scales all of the motor powers to stay
        // proportional and within power range
        double scalar = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        // Scalar equation above doesn't account for times when partial power is requested. This
        // compensates for that. And don't divide by 0
        if(drivePower != 0)
           scalar /= drivePower;

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

    // Truncates numbers to fit displays better. Not recommended for numbers that span many
    // magnitudes. Also consider the decimal point character.
    private String truncateNumber(double number, int length)
    {
        return Double.toString(number).substring(0,length);
    }

    // Used for calculating distances between 2 points
    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }
}
