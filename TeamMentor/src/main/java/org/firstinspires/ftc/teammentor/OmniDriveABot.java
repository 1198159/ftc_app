package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;


/**
 * Program used to control Drive-A-Bots.
 * This can be a good reference for drive controls.
 */
@TeleOp(name="Omni-Drive-A-Bot", group = "Swerve")
// @Disabled
public class OmniDriveABot extends LinearOpMode
{


    //Omnidrive motors have a particular location on the robot.
    //The location is used to calculate the motor's power when driving.
    private class OmniMotor
    {
        DcMotor motor;
        int x, y, rotation;

        public OmniMotor(DcMotor aMotor, int aX, int aY, int aRotation)
        {
            motor = aMotor;
            x = aX;
            y = aY;
            rotation = aRotation;

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public double calculatePower(double requestedX, double requestedY, double requestedRotation)
        {
            return    requestedRotation
                    + Math.signum(y) * requestedX  //y works as sine of the angle of each motor
                    + Math.signum(x) * requestedY; //x works as cosine of the angle of each motor
        }

        public void setPower(double power)
        {
            motor.setPower(power);
        }

        public double getPower()
        {
            return motor.getPower();
        }

    }

    OmniMotor motorFrontLeft = null;
    OmniMotor motorFrontRight = null;
    OmniMotor motorBackLeft = null;
    OmniMotor motorBackRight = null;


    // Drive direction constants
    //ToDo - add constants that let us switch which side of the robot is "front"
    public static enum DriveDirection
    {

    }

    @Override public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while(opModeIsActive())
        {
            driveOmniDrive( gamepad1.left_stick_x,    //local x motion power
                            gamepad1.left_stick_y,     //local y motion power
                            gamepad1.right_stick_x / 2); //divide rotation in half so we don't spin too quickly

            telemetry.update();
            idle();
        }
    }


    public void driveOmniDrive(double x, double y, double rotation)
    {
        double powFrontRight, powFrontLeft, powBackRight, powBackLeft;

        powFrontRight = motorFrontRight.calculatePower(x, y, rotation);
        powFrontLeft  = motorFrontLeft.calculatePower(x, y, rotation);
        powBackLeft   = motorBackLeft.calculatePower(x, y, rotation);
        powBackRight  = motorBackRight.calculatePower(x, y, rotation);

        //Find the maximum power applied to any motor
        double max = Math.max(Math.abs(powFrontLeft), Math.abs(powFrontRight));
        max = Math.max(max, Math.abs(powBackRight));
        max = Math.max(max, Math.abs(powBackLeft));

        //if any values were out of the motor power range of -1..1,
        // scale all of the values so they remain in proportion without overflowing
        if (max > 1.0)
        {
            powFrontRight *= 1.0/max;
            powFrontLeft  *= 1.0/max;
            powBackRight  *= 1.0/max;
            powBackLeft   *= 1.0/max;
        }

        //set the powers
        motorFrontLeft.setPower(powFrontLeft);
        motorFrontRight.setPower(powFrontRight);
        motorBackLeft.setPower(powBackLeft);
        motorBackRight.setPower(powBackRight);

    }

    public void initializeRobot()
    {
        // Initialize motors to be the hardware motors
        motorFrontLeft = new OmniMotor(hardwareMap.dcMotor.get("motorFrontLeft"), -1, -1, 315);
        motorFrontRight = new OmniMotor(hardwareMap.dcMotor.get("motorFrontRight"), 1,  -1,  45);
        motorBackLeft = new OmniMotor(hardwareMap.dcMotor.get("motorBackLeft"), -1,  1, 22);
        motorBackRight = new OmniMotor(hardwareMap.dcMotor.get("motorBackRight"), 1, 1, 135);

        // Set up telemetry data
        configureDashboard();
    }

    public void configureDashboard()
    {
        telemetry.addLine()
                .addData("Power | FrontLeft: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorFrontLeft.getPower());
                    }
                })
                .addData("FrontRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorFrontRight.getPower());
                    }
                })
                .addData("BackLeft: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBackLeft.getPower());
                    }
                })
                .addData("BackRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBackRight.getPower());
                    }
                });
    }

    public String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
}
