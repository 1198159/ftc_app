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

//TODO: add servo fork to configuration and make it move, look at old test code in team Swerve

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
    boolean isLegatoMode = false;
    boolean isRightBumperPushed = false;
    private ElapsedTime runtime = new ElapsedTime();
    Orientation angles;
    AvgFilter filterJoyStickInput = new AvgFilter();
    double startAngle;
    double currentAngle;
    double imuAngle;
    final double LIFT_POWER = 0.9;

    double motorLauncherSpeed = 0;
    double motorLauncherSetSpeed = 0;
    double driveSpeed = 0;

    double jx2;
    double jy2;
    double turn;


    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();
        servoForks.setPosition(0);

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

// move particle servo
           servoParticle.setPosition(Range.clip(-gamepad2.right_stick_y, 0, 0.95));
           servoForks.setPosition(gamepad2.left_stick_y);
           motorLauncherSpeed = Range.clip(gamepad2.right_trigger, 0, 0.7);
           motorLauncher.setPower(motorLauncherSpeed);
           //motorLauncher.setPower(Range.clip(gamepad2.right_trigger, 0, 0.7));

           // if just pressed and previous time wasn't pressed, for reverse mode
           if (gamepad1.left_bumper && !isLeftBumperPushed)
           {
               isLeftBumperPushed = true;
               isModeReversed = !isModeReversed;
           }
           isLeftBumperPushed = gamepad1.left_bumper;


// Adagio Legato Mode!
           if (isLegatoMode) // if mode activated, reduce constants and filter input
           {
               mecanumDrive(0.3, 0.3);
           }
           else
           {
               mecanumDrive(1.0, 0.8);
           }
/*
           if (gamepad2.a) // when game pad 2 a is held down, launcher motor runs
           {
               //rampShooterMotor(0.7);
               motorLauncher.setPower(0.7);
           }
           else
           {
               //rampShooterMotor(0);
               motorLauncher.setPower(0.0);
           }
*/
           if (gamepad2.dpad_left) // hold game pad left or right is held down
           {
               motorCollector.setPower(1.0);
           }
           else if (gamepad2.dpad_right)
           {
               motorCollector.setPower(-1.0);
           }
           else
           {
               motorCollector.setPower(0);
           }

           // press button a to toggle legato mode
           if (gamepad1.right_bumper && !isRightBumperPushed)
           {
               isRightBumperPushed = true;
               isLegatoMode = !isLegatoMode;
           }
           isRightBumperPushed = gamepad1.right_bumper;
           telemetry.addData("legato: ", isLegatoMode);

           telemetry.update();
           idle();
        } // =================================== end of while active ===========================

        // for safety, turn off all motors
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorLift.setPower(0);
        motorCollector.setPower(0);
    }

    public void mecanumDrive(double kDrive, double kPivot)
    {
        double avgX;
        double avgY;
        double avgPivot;

        double rx;  // represents RIGHT joystick "x axis"
        double ry;  // represents RIGHT joystick "y axis"
        double lx; // represents joystick LEFT "x axis"

        rx = gamepad1.right_stick_x;
        ry = -gamepad1.right_stick_y; // the joystick is reversed, so make this negative
        lx = gamepad1.left_stick_x;

        jx2 = modJoyStickInput(rx);
        jx2 = Range.clip(jx2, -1, 1);
        jy2 = modJoyStickInput(ry);
        jy2 = Range.clip(jy2, -1, 1);
        turn = modJoyStickInput(lx);
        turn = Range.clip(turn, -1, 1);

        filterJoyStickInput.appendInput(jx2, jy2, turn);

        if (isLegatoMode)
        {
            avgX = filterJoyStickInput.getFilteredX();
            avgY = filterJoyStickInput.getFilteredY();
            avgPivot = filterJoyStickInput.getFilteredP();
        }
        else
        {
            avgX = jx2;
            avgY = jy2;
            avgPivot = turn;
        }

        if (isModeReversed)
        {
            /*
            jx2 = -jx2;
            jy2 = -jy2;
            */
            avgX = -avgX;
            avgY = -avgY;
        }
        telemetry.addData("Reversed: ", isModeReversed);

        motorFrontLeft.setPower(avgX * kDrive + avgY * kDrive + avgPivot * kPivot);
        motorFrontRight.setPower(-avgX * kDrive + avgY * kDrive - avgPivot * kPivot);
        motorBackLeft.setPower(-avgX * kDrive + avgY * kDrive + avgPivot * kPivot);
        motorBackRight.setPower(avgX * kDrive + avgY * kDrive - avgPivot * kPivot);
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

    /*
    public void avgJoyStickInput()
    {
        jx[0] = jx2;
        jy[0] = jy2;
        jPivot[0] = turn;
        // shift of values
        for (int i = 1; i < jx.length; i++) // change numAvg later, has to be less than jx.length
        {
            jx[i] = jx[i - 1];
            jy[i] = jy[i - 1];
            jPivot[i] = jPivot[i - 1];
        }
        avgX = 0.0;
        avgY = 0.0;
        avgPivot = 0.0;
        // sum of values
        for (int i = 0; i < jx.length; i++)
        {
            avgX += jx[i];
            avgY += jx[i];
            avgPivot += jx[i];
        }
        avgX = avgX / (double) jx.length;
        avgY = avgY / (double) jy.length;
        avgPivot = avgPivot / (double) jPivot.length;
    }
    */

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
        motorCollector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // caution, no encoder

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // reverse front and back right motors just for TeleOp
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);

        motorLift.setDirection(DcMotor.Direction.REVERSE);
        motorCollector.setDirection(DcMotor.Direction.REVERSE);

        // Set up telemetry data
        configureDashboard();
    }

    public void rampShooterMotor(double speed)
    {
        while (motorLauncherSpeed < speed)
        {
            motorLauncherSpeed += speed / 20.0;
            motorLauncher.setPower(motorLauncherSpeed);
            sleep(200);
        }
        while (motorLauncherSpeed > speed)
        {
            motorLauncherSpeed -= 0.05;
            motorLauncher.setPower(motorLauncherSpeed);
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
    }

    public String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
}
