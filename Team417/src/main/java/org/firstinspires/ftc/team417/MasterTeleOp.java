package org.firstinspires.ftc.team417;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="TeleOp", group = "Swerve")
// @Disabled

// CONTROLS:
// left joystick controls the turn, or pivot alone
// right joystick controls forwards, backwards, horizontal left and right
// hold down right bumper for slow mode
// push left bumper to reverse drive

public class MasterTeleOp extends MasterOpMode
{
    boolean isSwitchPressed = false;
    boolean isBButtonPressed = false; // isBbuttonPressed causes is lift to toggle
    boolean isLiftActivated = false;
    boolean isLeftBumperPushed = false; // is left bumper causes is mode to toggle
    boolean isModeReversed = false;
    private ElapsedTime runtime = new ElapsedTime();
    Orientation angles;
    double startAngle;
    double currentAngle;
    double imuAngle;
    final double LIFT_POWER = 0.9;

    double motorLauncherSpeed = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        telemetry.addData("Path", "InitDone");
        telemetry.update();

        //startAngle = imu.getAngularOrientation().firstAngle;

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while (opModeIsActive())
       {
           /*
           imuAngle = imu.getAngularOrientation().firstAngle;
           currentAngle = imuAngle - startAngle;
           currentAngle = adjustAngles(currentAngle);
           */

           isSwitchPressed = liftSwitch.getState();    //  Read the input pin

           if (gamepad2.b && !isBButtonPressed)
            {
                isBButtonPressed = true;
                isLiftActivated = !isLiftActivated;
            }
            isBButtonPressed = gamepad2.b;

            if (isLiftActivated && !isSwitchPressed) // if the switch is NOT pressed
            {
                if (gamepad2.dpad_up)
                {
                    motorLift.setPower(LIFT_POWER);
                }
                else if (gamepad2.dpad_down)
                {
                    motorLift.setPower(-LIFT_POWER);
                }
                else
                {
                    motorLift.setPower(0);
                }
            }
           else
            {
                motorLift.setPower(0);
            }
            // Gamepads have a new state, so update things that need updating
            //if(updateGamepads())

           // if just pressed and previous time wasn't pressed
           if (gamepad1.left_bumper && !isLeftBumperPushed)
           {
               isLeftBumperPushed = true;
               isModeReversed = !isModeReversed;
           }
           isLeftBumperPushed = gamepad1.left_bumper;

           if (gamepad1.right_bumper) // if slow mode (when right bumper is held down)
            {
                mecanumDrive(0.5, 0.3);
            }
           else
           {
               mecanumDrive(1.0, 0.7);
           }

            telemetry.update();
            idle();



           if (gamepad2.right_bumper) // when right bumper is held down, launcher motor runs
           {
               rampShooterMotor(0.9);
           }
           else
           {
               rampShooterMotor(0);
           }

           telemetry.update();
           idle();
        }

        // for safety, turn off all motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorLift.setPower(0);
        //motorCollector.setPower(0);
    }

    public void mecanumDrive(double kDrive, double kPivot)
    {
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
        jx2 = Range.clip(jx2, -1, 1);
        jy2 = modJoyStickInput(ry);
        jy2 = Range.clip(jy2, -1, 1);
        turn = modJoyStickInput(lx);
        turn = Range.clip(turn, -1, 1);

        if (isModeReversed)
        {
            jx2 = -jx2;
            jy2 = -jy2;
        }
        telemetry.addData("Reversed: ", isModeReversed);

        motorFrontLeft.setPower(jx2 * kDrive + jy2 * kDrive + turn * kPivot);
        motorFrontRight.setPower(-jx2 * kDrive + jy2 * kDrive - turn * kPivot);
        motorBackLeft.setPower(-jx2 * kDrive + jy2 * kDrive + turn * kPivot);
        motorBackRight.setPower(jx2 * kDrive + jy2 * kDrive - turn * kPivot);
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
        // run to position mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //motorCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // caution, no encoder

        // reverse front and back right motors just for TeleOp
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);

        motorLift.setDirection(DcMotor.Direction.REVERSE);

        // Set up telemetry data
        configureDashboard();
    }

    public void rampShooterMotor(double speed)
    {
        while (motorLauncherSpeed < speed)
        {
            motorLauncherSpeed += speed / 10.0;
            //motorLauncher.setPower(motorLauncherSpeed);
            sleep(200);
        }
        while (motorLauncherSpeed > speed)
        {
            motorLauncherSpeed -= speed / 10.0;
            //motorLauncher.setPower(motorLauncherSpeed);
            sleep(200);
        }
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
                });

/*
        telemetry.addLine()
                .addData("currentAnglet: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(currentAngle);
                    }
                })
                .addData("startAngle: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(startAngle);
                    }
                })
                .addData("imuAngle: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(imuAngle);
                    }
                });
*/

        /*
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, currentAngle);
                    }
                })
                .addData("startAngle", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, startAngle);
                    }
                })
                .addData("imuAngle", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, imuAngle);
                    }
                });
*/
    }

    public String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
}
