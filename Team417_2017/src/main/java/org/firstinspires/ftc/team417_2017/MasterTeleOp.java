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

    /*
    This keeps track of the side or corner of the robot that is considered "forwards" by the driver.
    0 degrees is the side of the robot with with the phone, and since there are eight possible
    fronts, we add 45 degrees to 0 as we rotate the robot to the right.
     */
    int frontAngle = 0;

    // angle is the arc tangent of (-jx/jy), considering that 0 degrees is forwards
    double driveAngle; // used to calculate the drive angle based on the x and y position on the joystick
    double y;
    double x;
    double power;
    double pivotPower;
    double angle;

    boolean isModeReversed = false;
    boolean isLegatoMode = false;
    boolean isRightBumperPushed = false;
    private ElapsedTime runtime = new ElapsedTime();
    AvgFilter filterJoyStickInput = new AvgFilter();

    void omniDriveTeleOp()
    {
        /*
        // if the left bumper is pushed and it hasn't been pushed before,
        if (gamepad1.left_bumper && !isLeftBumperPushed)
        {
            isLeftBumperPushed = true;
            frontAngle -= 45; // shift the front 45 degrees to the left
        }
        isLeftBumperPushed = gamepad1.left_bumper;

        // if the right bumper is pushed and it hasn't been pushed before,
        if (gamepad1.right_bumper && !isRightBumperPushed)
        {
            isRightBumperPushed = true;
            frontAngle += 45; // shift the front 45 degrees to the right
        }
        isRightBumperPushed = gamepad1.right_bumper;
        */

        y = -gamepad1.right_stick_y; // Y axis is negative when up
        x = gamepad1.right_stick_x;
        pivotPower = gamepad1.left_stick_x;

        // calculate the power for each motor

        powerFL = x + y + pivotPower;
        powerFR = -x + y - pivotPower;
        powerBL = -x + y + pivotPower;
        powerBR = x + y - pivotPower;

/*
        powerFL = px + 0*py + pivotPower;
        powerFR = 0*px + py - pivotPower;
        powerBL = 0*px + py + pivotPower;
        powerBR = px + 0*py - pivotPower;
*/

        // set power to the motors
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    public double modJoyStickInput(double i) // i is the raw joystick input
    {
        return Math.pow(i,2) * Math.signum(i);
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

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
