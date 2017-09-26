package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

//@TeleOp(name="TeleOp", group = "Swerve")
//@Disabled

abstract public class MasterTeleOp extends MasterOpMode
{
    boolean isLeftBumperPushed = false;
    boolean isLeftPusherUp = false;
    boolean isRightPusherUp = false;
    boolean isXButtonPressed = false;
    boolean isYButtonPressed = false;

    // angle is the arc tangent of (-jx/jy), considering that 0 degrees is forwards
    double driveAngle; // used to calculate the drive angle based on the x and y position on the joystick

    boolean isModeReversed = false;
    boolean isLegatoMode = false;
    boolean isRightBumperPushed = false;
    private ElapsedTime runtime = new ElapsedTime();
    AvgFilter filterJoyStickInput = new AvgFilter();

    void omniDriveTeleOp()
    {
        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = gamepad1.left_stick_x;
        double power = calcDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward

        omniDrive(angle, power, turnPower);
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

        motorFL.setPower(avgX * kDrive + avgY * kDrive + avgPivot * kPivot);
        motorFR.setPower(-avgX * kDrive + avgY * kDrive - avgPivot * kPivot);
        motorBL.setPower(-avgX * kDrive + avgY * kDrive + avgPivot * kPivot);
        motorBR.setPower(avgX * kDrive + avgY * kDrive - avgPivot * kPivot);
        // lx is defined as game pad input, then turn gets value from function "modJoyStickInput"
        // turn used in final equation for each motor
    }

    public double modJoyStickInput(double x) // x is the raw joystick input, refer to "modJoyStickInput"
    {
        return Math.pow(x,2) * Math.signum(x);
    }

    public void initializeRobot()
    {
        // Initialize motors to be the hardware motors
        super.initializeHardware(); // comment out this line if blank config
        // run to position mode
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // reverse front and back right motors just for TeleOp
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);

        // Set up telemetry data
        configureDashboard();
    }


    public void configureDashboard()
    {
        telemetry.addLine()
                .addData("Power | FrontLeft: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorFL.getPower());
                    }
                })
                .addData("Power | FrontRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorFR.getPower());
                    }
                })
                .addData("Power | BackLeft: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBL.getPower());
                    }
                })
                .addData("Power | BackRight: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBR.getPower());
                    }
                });
    }

    public String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
}
