package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;

@TeleOp(name = "SummerRowCinco", group = "TeleOp")
public class SummerRowCinco extends LinearOpMode
{
    // Omnidrive motors have a particular location on the robot.
    // The location is used to calculate the motor's power when driving.
    private class OmniMotor
    {
        DcMotor motor;
        int x, y, rotation;

        private OmniMotor(DcMotor aMotor, int aX, int aY, int aRotation)
        {
            motor = aMotor;
            x = aX;
            y = aY;
            rotation = aRotation;   // todo aRotation is not used in any calculations, so it seems unnecessary.  It would be a good idea to reformat the way the motors are dealt with, since this is not a great way of doing it

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        double calculatePowerOmniXConfiguration(double requestedX, double requestedY, double requestedRotation)
        {
            return  requestedRotation
                    + Math.signum(y) * requestedX
                    + Math.signum(x) * requestedY;
        }

        private void setPower(double power)
        {
            motor.setPower(power);
        }

        private double getPower()
        {
            return motor.getPower();
        }
    }


    // Create motors
    OmniMotor motor1;
    OmniMotor motor2;
    OmniMotor motor3;
    OmniMotor motor4;
    DcMotor collectorMotor;
    DcMotor slideMotor;


    private void driveOmniDrive(double x, double y, double rotation)
    {
        double power1 = 0, power2 = 0, power3 = 0, power4 = 0;

        // Calculate motor powers based on driver input and their locations
        power1 = motor1.calculatePowerOmniXConfiguration(x, y, rotation);
        power2 = motor2.calculatePowerOmniXConfiguration(x, y, rotation);
        power3 = motor3.calculatePowerOmniXConfiguration(x, y, rotation);
        power4 = motor4.calculatePowerOmniXConfiguration(x, y, rotation);


        // Find the maximum power applied to any motor
        double max = Math.max(Math.abs(power1), Math.abs(power2));
        max = Math.max(max, Math.abs(power3));
        max = Math.max(max, Math.abs(power4));

        // If any values were out of the motor power range of -1 to 1,
        // scale all of the values so they remain in proportion without overflowing
        if (max > 1.0)
        {
            power1 *= 1.0 / max;
            power2 *= 1.0 / max;
            power3 *= 1.0 / max;
            power4 *= 1.0 / max;
        }

        //set the powers
        motor1.setPower(power1);
        motor2.setPower(power2);
        motor3.setPower(power3);
        motor4.setPower(power4);
    }


    private void initializeRobot()
    {
        // Initialize motors to be the hardware motors
        // configuration for "X"
        // adjust these 1's and -1's if your robot doesn't move in the direction you like
        motor1 = new OmniMotor(hardwareMap.dcMotor.get("motor1"),  1,   1, 315);    // todo See above
        motor2 = new OmniMotor(hardwareMap.dcMotor.get("motor2"),  1,  -1,  45);
        motor3 = new OmniMotor(hardwareMap.dcMotor.get("motor3"), -1,  -1, 135);
        motor4 = new OmniMotor(hardwareMap.dcMotor.get("motor4"), -1,   1, 225);

        // Initialitze motors that operate collector and linear slide
        collectorMotor = hardwareMap.dcMotor.get("collectorMotor");
        slideMotor = hardwareMap.dcMotor.get("slideMotor");

        // Set up telemetry data
        configureDashboard();
    }


    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while(opModeIsActive())
        {

            driveOmniDrive( gamepad1.left_stick_x,    // local x motion power
                    gamepad1.left_stick_y,     // local y motion power
                    -gamepad1.right_stick_x / 2); // divide rotation in half so we don't spin too quickly

            if(gamepad2.dpad_up)
            {
                collectorMotor.setPower(0.2);
            }
            else if(gamepad2.dpad_down)
            {
                collectorMotor.setPower(-0.2);
            }
            else if(gamepad2.dpad_left)
            {
                collectorMotor.setPower(0);
            }
            else if(gamepad2.a)
            {
                slideMotor.setPower(0.2);
            }
            else if(gamepad2.b)
            {
                slideMotor.setPower(-0.2);
            }
            else if(gamepad2.x)
            {
                slideMotor.setPower(0);
            }

            telemetry.update();
            idle();
        }
    }


    private void configureDashboard()
    {
        telemetry.addLine()
                .addData("Power | 1: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor1.getPower());
                    }
                })
                .addData("2: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor2.getPower());
                    }
                })
                .addData("3: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor3.getPower());
                    }
                })
                .addData("4: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor4.getPower());
                    }
                });
    }

    private String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
}
