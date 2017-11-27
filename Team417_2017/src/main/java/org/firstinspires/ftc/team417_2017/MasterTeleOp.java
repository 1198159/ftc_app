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

    /*
    This keeps track of the side or corner of the robot that is considered "forwards" by the driver.
    0 degrees is the side of the robot with with the phone, and since there are eight possible
    fronts, we add 45 degrees to 0 as we rotate the robot to the right.
     */
    int frontAngle = 0;

    double anglePivot;
    double robotAngle;
    double jpivot;
    double kAngle;
    double pivot;
    double error;

    // angle is the arc tangent of (-jx/jy), considering that 0 degrees is forwards
    double driveAngle; // used to calculate the drive angle based on the x and y position on the joystick
    double y;
    double x;
    double power;
    double pivotPower;
    double angle;
    final double ADAGIO_POWER = 0.45;

    boolean isModeReversed = true;
    boolean isLegatoMode = false;
    boolean isLeftBumperPushed = true;
    boolean isRightBumperPushed = false;
    private ElapsedTime runtime = new ElapsedTime();
    AvgFilter filterJoyStickInput = new AvgFilter();

    void omniDriveTeleOp()
    {
        // press button right bumper to toggle adagio legato mode
        if (gamepad1.right_bumper && !isRightBumperPushed)
        {
            isRightBumperPushed = true;
            isLegatoMode = !isLegatoMode;
        }
        isRightBumperPushed = gamepad1.right_bumper;
        telemetry.addData("legato: ", isLegatoMode);

        if (isLegatoMode) // Legato Mode
        {
            y = -Range.clip(gamepad1.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
            x = Range.clip(gamepad1.right_stick_x, -ADAGIO_POWER, ADAGIO_POWER);
            pivotPower = Range.clip(gamepad1.left_stick_x, -0.35, 0.35);
        }
        else // Staccato Mode
        {
            y = -gamepad1.right_stick_y; // Y axis is negative when up
            x = gamepad1.right_stick_x;
            pivotPower = Range.clip(gamepad1.left_stick_x, -0.8, 0.8);
        }


        // Reverse mode, activated by GamePad1's left bumper
        if (gamepad1.left_bumper && !isLeftBumperPushed)
        {
            isLeftBumperPushed = true;
            isModeReversed = !isModeReversed;
        }
        isLeftBumperPushed = gamepad1.left_bumper;
        telemetry.addData("Reverse: ", isModeReversed);
        telemetry.update();

        if (isModeReversed)
        {
            // calculate the power for each motor
            powerFL = -x - y + pivotPower;
            powerFR = x - y - pivotPower;
            powerBL = x - y + pivotPower;
            powerBR = -x - y - pivotPower;
        }
        else
        {
            // calculate the power for each motor
            powerFL = x + y + pivotPower;
            powerFR = -x + y - pivotPower;
            powerBL = -x + y + pivotPower;
            powerBR = x + y - pivotPower;
        }

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

    void imuOmniTeleOp()
    {
        y = -gamepad1.right_stick_y; // Y axis is negative when up
        x = gamepad1.right_stick_x;
        pivotPower = Range.clip(gamepad1.left_stick_x, -0.8, 0.8);

        robotAngle = imu.getAngularOrientation().firstAngle;

        anglePivot = 2 * (anglePivot - jpivot);
        anglePivot = adjustAngles(anglePivot);

        kAngle = 0.035;
        //robotAngle = adjustAngles(robotAngle);
        error = robotAngle - anglePivot;
        error = adjustAngles(error);
        pivot = error * kAngle;

        // calculate the power for each motor
        powerFL = x + y + pivotPower;
        powerFR = -x + y - pivotPower;
        powerBL = -x + y + pivotPower;
        powerBR = x + y - pivotPower;

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
