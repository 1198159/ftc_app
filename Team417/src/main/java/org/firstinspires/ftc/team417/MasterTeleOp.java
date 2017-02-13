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
    final double LIFT_POWER = 1.0;

    int liftState = 0;
    boolean isLiftRunning = false;
    int liftZero = 0;
    int liftZero2 = 0;
    ElapsedTime liftTimer = new ElapsedTime();

    double motorLauncherSpeed = 0;
    double motorLauncherSetSpeed = 0;
    double driveSpeed = 0;

    double jx2;  //CodeReview: This variable is only used in the MecanumDrive method. It should be declared inside that method
    double jy2;  //CodeReview: (same comment as above)
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

           //CodeReview: Jarrod mentioned that your robot drives sluggishly. That might suggest
           //  that your main loop should be more efficient. To test that theory, you might try adding
           //  a timer + some telemetry to show you how long your main loop takes to execute on average.
           //  If it seems like it's taking a long time per loop, you could look for ways to optimize.
           //  E.g. you might be spending a lot of time updating your filter, or doing the Math.pow(x,2) command.
           //  You could comment out various parts of your code to see how it affects the loop time.

           if (gamepad2.a)
           {
               motorLift.setPower(-gamepad2.left_stick_y);
               motorLift2.setPower(-gamepad2.left_stick_y);
           }
           else if(!isLiftRunning)
           {
               motorLift.setPower(0);
               motorLift2.setPower(0);
           }


           if (gamepad2.b && gamepad2.left_bumper)
           {
               liftState = 0;
               isLiftRunning = true;
           }

           if (liftState == 0 && isLiftRunning)
           {
               motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               motorLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               liftZero = motorLift.getCurrentPosition();
               liftZero2 = motorLift2.getCurrentPosition();
               motorLift.setTargetPosition(liftZero + (int) (1.0 * COUNTS_PER_LIFT_MOTOR_REV));
               motorLift2.setTargetPosition(liftZero2 + (int) (1.0 * COUNTS_PER_LIFT_MOTOR_REV));
               motorLift.setPower(0.8);
               motorLift2.setPower(0.8);
               liftState++;
               idle();
           }
           else if (liftState == 1 && isLiftRunning)
           {
               telemetry.addData("Motor lift busy: %b", motorLift.isBusy());
               telemetry.addData("lift running: %b", isLiftRunning);
               telemetry.addData("lift cur pos: %d", motorLift.getCurrentPosition());
               telemetry.addData("lift tar pos: %d", motorLift.getTargetPosition());
               telemetry.addData("lift tar pos: %f", motorLift.getPower());

               if (!motorLift.isBusy() && !motorLift2.isBusy())
               {
                   motorLift.setPower(0);
                   motorLift2.setPower(0);
                   liftTimer.reset();
                   liftState++;
               }
               idle();
           }
           else if (liftState == 2 && isLiftRunning)
           {
               if (liftTimer.milliseconds() > 600)
               {
                   liftState++;
               }
               idle();
           }
           else if (liftState == 3 && isLiftRunning)
           {
               motorLift.setTargetPosition(liftZero);
               motorLift2.setTargetPosition(liftZero2);
               motorLift.setPower(0.8);
               motorLift2.setPower(0.8);
               liftState++;
               idle();
           }
           else if (liftState == 4 && isLiftRunning)
           {
               if (!motorLift.isBusy() && !motorLift2.isBusy())
               {
                   motorLift.setPower(0);
                   motorLift2.setPower(0);
                   motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                   motorLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                   isLiftRunning = false;
                   liftState++;
               }
               idle();
           }



// move particle servo
           servoParticle.setPosition(Range.clip(-gamepad2.right_stick_y, 0, 0.7));
           servoForks.setPosition(gamepad2.left_stick_y);

           // HIGH SPEED FOR MOTOR LAUNCHER IS 0.7, LOW SPEED IS 0.4 (held at least half way down)
           if (gamepad2.right_trigger > 0.5) motorLauncherSetSpeed = 0.8;
           else if (gamepad2.left_trigger > 0.5) motorLauncherSetSpeed = 0.6;
           else motorLauncherSetSpeed = 0;

           //CodeReview: come talk with me about the following block of code.
           // There's almost certainly a bug here, or something unintended,
           // and there might be a simpler way to do this.
           if (runtime.milliseconds() > 100)
           {
               resetStartTime(); //CodeReview: Come talk with me. I think this method is doing something other than what you want.
                                 // It's very unlikely that you would really want to call this method in your opmode.
               if (motorLauncherSpeed < motorLauncherSetSpeed)
               {
                   motorLauncherSpeed += 0.1;
               }
               else if (motorLauncherSpeed > motorLauncherSetSpeed) // shouldn't go negative
               {
                   if (motorLauncherSpeed < 0.1) motorLauncherSpeed = 0.0;
                   else motorLauncherSpeed -= 0.01;
               }
           }
           motorLauncher.setPower(Range.clip(motorLauncherSpeed, 0, 0.8));


           //motorLauncher.setPower(Range.clip(gamepad2.right_trigger, 0, 0.7));

           // if just pressed and previous time wasn't pressed, for reverse mode
           if (gamepad1.left_bumper && !isLeftBumperPushed)
           {
               isLeftBumperPushed = true;
               isModeReversed = !isModeReversed;
           }
           isLeftBumperPushed = gamepad1.left_bumper;


// Adagio Legato Mode!
           //CodeReview: you don't process the buttons that turn this mode on until later down,
           //  which means that the response to turning on LegatoMode is delayed by one loop.
           //  If you move that code above this block, it will be more responsive.
           if (isLegatoMode) // if mode activated, reduce constants and filter input
           {
               mecanumDrive(0.3, 0.3);
           }
           else
           {
               mecanumDrive(1.0, 0.8);
           }

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
        motorLift2.setPower(0);
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

        //CodeReview: put this next line inside the (isLegatoMode) block below,
        //  so you only pay for updating the filter when you're actually using it.
        //  (I *think* you're only using it in legato mode...)
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

    public void initializeRobot()
    {
        // Initialize motors to be the hardware motors
        super.initializeHardware(); // comment out this line if blank config
        // run to position mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        // TODO: check if the new motor have to be reversed or not
        motorLift.setDirection(DcMotor.Direction.REVERSE);
        //motorLift2.setDirection(DcMotor.Direction.REVERSE);
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

    public void RunDistance (double speed, double distInches, double timeout) throws InterruptedException
    {
        int newTarget;

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
                .addData("Lift | State: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(liftState);
                    }
                });
    }

    public String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
}
