package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;


/**
 * Program not used to control Drive-A-Bots.
 * This can be a good reference for drive controls.
 */
@TeleOp(name="Mecanum Drive", group = "Swerve")
// @Disabled
public class MasterTeleOp extends MasterOpMode
{
    boolean isLiftActivated = false;
    private ElapsedTime runtime = new ElapsedTime();
    boolean isSwitchPressed = false;


    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while (opModeIsActive())
       {
           isSwitchPressed = digIn.getState();    //  Read the input pin
            if (gamepad1.b)
            {
                if (isLiftActivated == false)
                {
                    isLiftActivated = true;
                    runtime.reset();
                }
                else if (runtime.seconds() > 1) // timeout is one second
                {
                    isLiftActivated = false;
                }
            }

            if (gamepad1.dpad_up && isLiftActivated && !isSwitchPressed) // if the switch is NOT pressed
            {
                motorLift.setPower(0.5f);
            }
           else
            {
                motorLift.setPower(0);
            }
            // Gamepads have a new state, so update things that need updating
            //if(updateGamepads())
            {
                mecanumDrive();
            }

            telemetry.update();
            idle();
        }
    }

    /*
     * Controls the robot with two joysticks
     * Left joystick controls the turn / pivot
     * Right joystick controls the forwards, backwards, left and right
     */

    public void mecanumDrive()
    {
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double rx;  // represents RIGHT joystick "x axis"
        double ry;  // represents RIGHT joystick "y axis"
        double turn; // for turning with LEFT joystick
        double lx; // represents joystick LEFT "x axis"

        rx = gamepad1.right_stick_x;
        ry = -gamepad1.right_stick_y; // the joystick is reversed, so make this negative
        lx = gamepad1.left_stick_x;
        double jx2; // jx2 and jy2 are the modified variables to the quadratic function
        double jy2;

        jx2 = modJoyStickInput(rx);
        jy2 = modJoyStickInput(ry);
        turn = modJoyStickInput(lx);

        motorFrontLeft.setPower(jx2 + jy2 + turn/2);
        motorFrontRight.setPower(-jx2 + jy2 - turn/2);
        motorBackLeft.setPower(-jx2 + jy2 + turn/2);
        motorBackRight.setPower(jx2 + jy2 - turn/2);
        // lx is defined as game pad input, then turn gets value from function "modJoyStickInput"
        // turn used in final equation for each motor
    }


    /* TABLE:

                 FL      FR      BL      BR
    rotate ->    +        -      +        -
    rotate <-    -        +      -        +
    forward      +        +      +        +
    backward     -        -      -        -
    left         -        +      +        -
    right        +        -      -        +
    d. left      0        +      +        0
    d. right     +        0      0        +


    */

    public double modJoyStickInput(double x) // x is the raw joystick input, refer to "modJoyStickInput"
    {
        return Math.pow(x,2) * Math.signum(x);
    }


    public void initializeRobot()
    {
        // Initialize motors to be the hardware motors
        super.initializeHardware(); // comment out this line if blank config
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
                .addData("Power | FrontRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorFrontRight.getPower());
                    }
                })
                .addData("Power | BackLeft: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBackLeft.getPower());
                    }
                })
                .addData("Power | BackRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBackRight.getPower());
                    }
                })

        ;

    }

    public String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
}
