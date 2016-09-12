package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class contains all objects and methods that should be accessible by all OpModes
 */
public class Master extends LinearOpMode
{

    // Declares all hardware on robot
    DcMotor motorFL = null;
    DcMotor motorFR = null;
    DcMotor motorBL = null;
    DcMotor motorBR = null;

    // Initialize hardware on robot
    public void initHardware()
    {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
    }

    // Sends information to Driver Station screen for drivers to see
    public void sendTelemetry()
    {
        //TODO: Probably won't need this after testing. It takes up a lot of room, so remove if no longer needed.
        // Drive motor info
        telemetry.addData("FL", truncateNumber(motorFL.getPower(), 4));
        telemetry.addData("FR", truncateNumber(motorFR.getPower(), 4));
        telemetry.addData("BL", truncateNumber(motorBL.getPower(), 4));
        telemetry.addData("BR", truncateNumber(motorBR.getPower(), 4));

        telemetry.update();
    }

    // Truncates numbers to fit displays better. Not recommended for numbers that span many
    // magnitudes. Also consider the decimal point character.
    public String truncateNumber(double number, int length)
    {
        return Double.toString(number).substring(0,length);
    }

    // Used for calculating distances between 2 points
    public double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    // TODO: Is it possible for us to remove this from here without breaking stuff?
    @Override
    public void runOpMode() throws InterruptedException {}
}
