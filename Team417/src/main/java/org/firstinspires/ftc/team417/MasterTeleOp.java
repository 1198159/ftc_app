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

public class MasterTeleOp extends MasterOpMode
{
    boolean isLeftBumperPushed = false;
    boolean isLeftPusherUp = false;
    boolean isRightPusherUp = false;
    boolean isXButtonPressed = false;
    boolean isYButtonPressed = false;

    boolean isModeReversed = false;
    boolean isLegatoMode = false;
    boolean isRightBumperPushed = false;
    private ElapsedTime runtime = new ElapsedTime();
    AvgFilter filterJoyStickInput = new AvgFilter();

    // For autonomous lift
    int liftState = 0;
    boolean isLiftRunning = false;
    int liftZero = 0;
    int liftZero2 = 0;
    ElapsedTime liftTimer = new ElapsedTime();

    double motorLauncherSpeed = 0;
    double motorLauncherSetSpeed = 0;


    @Override
    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        telemetry.addData("Path", "InitDone");
        telemetry.update();

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while (opModeIsActive())
       {
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


// Autonomous Fork Deploy
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
               motorLift.setTargetPosition(liftZero + (int) (1.1 * COUNTS_PER_LIFT_MOTOR_REV));
               motorLift2.setTargetPosition(liftZero2 + (int) (1.1 * COUNTS_PER_LIFT_MOTOR_REV));
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



// Move particle servo
           if (-gamepad2.right_stick_y > 0.4) servoParticle.setPosition(SERVO_PARTICLE_HIGH);
           else servoParticle.setPosition(SERVO_PARTICLE_LOW);

           // HIGH SPEED FOR MOTOR LAUNCHER IS 0.7, LOW SPEED IS 0.4 (held at least half way down)
           if (gamepad2.right_trigger > 0.5) motorLauncherSetSpeed = 0.8;
           else if (gamepad2.left_trigger > 0.5) motorLauncherSetSpeed = 0.6;
           else motorLauncherSetSpeed = 0;

// Ramp up flywheel motor
           if (runtime.milliseconds() > 100)
           {
               runtime.reset();

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
           if (gamepad2.left_bumper && motorLauncherSpeed < 0.1) // if speed is low and bumper activated
           {
               motorLauncher.setPower(-0.2); // then go backwards (slowly)
           }
           else motorLauncher.setPower(Range.clip(motorLauncherSpeed, 0, 0.8));


// press button A to toggle adagio legato mode
           if (gamepad1.right_bumper && !isRightBumperPushed)
           {
               isRightBumperPushed = true;
               isLegatoMode = !isLegatoMode;
           }
           isRightBumperPushed = gamepad1.right_bumper;
           telemetry.addData("legato: ", isLegatoMode);
           telemetry.update();
           idle();

// Reverse mode, activated by GamePad1's left bumper
           if (gamepad1.left_bumper && !isLeftBumperPushed)
           {
               isLeftBumperPushed = true;
               isModeReversed = !isModeReversed;
           }
           isLeftBumperPushed = gamepad1.left_bumper;


// Adagio Legato Mode!  If mode activated, reduce constants and filter input (uses filter input class)
           if (isLegatoMode) mecanumDrive(0.3, 0.3);
           else mecanumDrive(1.0, 0.8);


// Operate collector motor (hold gamepad left (out) or right (in)
           if (gamepad2.dpad_left) motorCollector.setPower(1.0);
           else if (gamepad2.dpad_right) motorCollector.setPower(-1.0);
           else motorCollector.setPower(0);


// Button pusher servo: press button x to toggle up and down,
           if (gamepad2.x && !isXButtonPressed)
           {
               isXButtonPressed = true;
               isLeftPusherUp = !isLeftPusherUp;
           }
           isXButtonPressed = gamepad2.x;
// Press button y to toggle up and down
           if(isLeftPusherUp) servoLeftPusher.setPosition(LEFT_PUSHER_HIGH);
           else servoLeftPusher.setPosition(LEFT_PUSHER_LOW);

           if (gamepad2.y && !isYButtonPressed)
           {
               isYButtonPressed = true;
               isRightPusherUp = !isRightPusherUp;
           }
           isYButtonPressed = gamepad2.y;

           if(isRightPusherUp) servoRightPusher.setPosition(RIGHT_PUSHER_HIGH);
           else servoRightPusher.setPosition(RIGHT_PUSHER_LOW);

        } // =================================== end of while op mode is active ===========================

// For safety, turn off all motors
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
        double jx2;
        double jy2;
        double turn;

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

        // used in all modes including adagio legato mode
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


    /* MECANUM DRIVE TABLE:
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
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLift.setDirection(DcMotor.Direction.FORWARD);
        motorCollector.setDirection(DcMotor.Direction.FORWARD);
        motorLauncher.setDirection(DcMotor.Direction.REVERSE);

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
