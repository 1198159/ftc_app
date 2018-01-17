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
    double y = 0;
    double x = 0;
    double pivotPower = 0;
    final double ADAGIO_POWER = 0.3;

    boolean isModeReversed = true;
    boolean isLegatoMode = false;
    boolean isCornerDrive = false;
    boolean driveClose = false;
    boolean driveOpen = false;

    int ggOpenState;
    int ggCloseState;

    AvgFilter filterJoyStickInput = new AvgFilter();

    // declare variables for the GG (AndyMarkNeveRest 3.7 motor is 44.4 counts per rev)
    double KGlyph = 1.0f/600.0f; // constant for proportional drive
    double MINSPEED = 0.05;
    double MAXSPEED = 0.4;
    double speedGG; // current speed
    int errorMaxGG; // error for closing grabber
    int errorMinGG; // error for opening grabber

    int curGGPos;
    int minGGPos = -180; // a bit less than the original starting position of zero (where we start it) (OPEN is more positive)
    int maxGGPos = -577; // maxGGPos equals the # rev to close/open GG (13 rev) times 44.4 counts per rev (CLOSED is more negative)

    void omniDriveTeleOp()
    {
        // hold right trigger for adagio legato mode
        if (gamepad1.right_trigger > 0.0 && !isCornerDrive)
            isLegatoMode = true;
        else
            isLegatoMode = false;
        telemetry.addData("legato: ", isLegatoMode);

        // hold left trigger for corner drive
        if (gamepad1.left_trigger > 0.0 && !isLegatoMode)
            isCornerDrive = true;
        else
            isCornerDrive = false;
        telemetry.addData("corner: ", isCornerDrive);

        if (isLegatoMode) // Legato Mode
        {
            y = -Range.clip(gamepad1.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
            x = Range.clip(gamepad1.right_stick_x, -ADAGIO_POWER, ADAGIO_POWER);
            pivotPower = Range.clip(gamepad1.left_stick_x, -0.3, 0.3);
            //pivotPower = (gamepad1.left_stick_x) * 0.3;
        }
        else if (isCornerDrive) // Corner drive
        {
            y = -Range.clip(gamepad1.right_stick_y, -0.6, 0.6); // Y axis is negative when up
            x = Range.clip(gamepad1.right_stick_x, -0.6, 0.6);
            pivotPower = Range.clip(gamepad1.left_stick_x, -0.6, 0.6);
            //pivotPower = (gamepad1.left_stick_x) * 0.6;
        }
        else // Staccato Mode
        {
            y = -gamepad1.right_stick_y; // Y axis is negative when up
            x = gamepad1.right_stick_x;
            pivotPower = Range.clip(gamepad1.left_stick_x, -0.9, 0.9);
            //pivotPower = (gamepad1.left_stick_x) * 0.9;
        }

        filterJoyStickInput.appendInput(x, y, pivotPower);
        x = filterJoyStickInput.getFilteredX();
        y = filterJoyStickInput.getFilteredY();
        pivotPower = filterJoyStickInput.getFilteredP();

/*
        // Reverse mode, activated by GamePad1's left bumper
        if (gamepad1.left_bumper && !isLeftBumperPushed)
        {
            isLeftBumperPushed = true;
            isModeReversed = !isModeReversed;
        }
        isLeftBumperPushed = gamepad1.left_bumper;
        telemetry.addData("Reverse: ", isModeReversed);
        telemetry.update();
*/

        if (isCornerDrive)
        {
            // corner drive
            powerFL = -y + pivotPower;
            powerFR = x - pivotPower;
            powerBL = x + pivotPower;
            powerBR = -y - pivotPower;
        }
        else if (isModeReversed)
        {
            // calculate the power for each motor (this is the default 'forward' in TeleOp)
            powerFL = -x - y + pivotPower;
            powerFR = x - y - pivotPower;
            powerBL = x - y + pivotPower;
            powerBR = -x - y - pivotPower;
        }
        else
        {
            // reverse mode
            powerFL = x + y + pivotPower;
            powerFR = -x + y - pivotPower;
            powerBL = -x + y + pivotPower;
            powerBR = x + y - pivotPower;
        }

        // Glyph lift up/down
        if(-gamepad2.right_stick_y < 0.0) // down
        {
            motorGlyphLeft.setPower(Range.clip(-gamepad2.right_stick_y, -0.65, -0.2));
            motorGlyphRight.setPower(Range.clip(-gamepad2.right_stick_y, -0.65, -0.2));
        }
        else if(-gamepad2.right_stick_y > 0.0) // up
        {
            motorGlyphLeft.setPower(Range.clip(-gamepad2.right_stick_y, 0.2, 0.65));
            motorGlyphRight.setPower(Range.clip(-gamepad2.right_stick_y, 0.2, 0.65));
        }
        else // turn motors off
        {
            motorGlyphLeft.setPower(0.0);
            motorGlyphRight.setPower(0.0);
        }
        telemetry.addData("GlyphLeftPos: ", motorGlyphLeft.getCurrentPosition());
        telemetry.addData("GlyphRightPos: ", motorGlyphRight.getCurrentPosition());
        telemetry.update();



        // Manual Glyph grabber open/close
        if (!driveOpen && !driveClose)
            curGGPos = motorGlyphGrab.getCurrentPosition(); // set the current position of the GG
        if(gamepad2.right_bumper && curGGPos > maxGGPos) // CLOSE (counter goes negative when closing)
        {
            motorGlyphGrab.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            errorMaxGG = curGGPos - maxGGPos;
            speedGG = Math.abs(errorMaxGG * KGlyph);
            speedGG = Range.clip(speedGG, MINSPEED, MAXSPEED);
            speedGG = speedGG * Math.signum(errorMaxGG);
            motorGlyphGrab.setPower(speedGG);
        }
        else if(gamepad2.left_bumper && curGGPos < minGGPos) // OPEN (counter goes positive when opening)
        {
            motorGlyphGrab.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            errorMinGG = curGGPos - minGGPos;
            speedGG = Math.abs(errorMinGG * KGlyph);
            speedGG = Range.clip(speedGG, MINSPEED, MAXSPEED);
            speedGG = speedGG * Math.signum(errorMinGG);
            motorGlyphGrab.setPower(speedGG);
        }
        else if (!gamepad2.left_bumper && !gamepad2.right_bumper && !driveOpen && !driveClose) // turn motor off
        {
            motorGlyphGrab.setPower(0.0);
        }
        telemetry.addData("GlyphGrabPos: ", motorGlyphGrab.getCurrentPosition());
        telemetry.update();


        // Button GG Open and Close!
        // Glyph Grabber drive to position close
        if(gamepad2.dpad_right)
        {
            driveClose = true;
            ggCloseState = 0;
            driveOpen = false;
            idle();
        }
        else if (driveClose && ggCloseState == 0)
        {
            motorGlyphGrab.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            curGGPos = motorGlyphGrab.getCurrentPosition();
            telemetry.addData("motorGGCounts:", curGGPos);
            telemetry.update();

            // calculate the speed of the GG motor
            errorMaxGG = curGGPos - maxGGPos;
            speedGG = Math.abs(errorMaxGG * KGlyph);
            speedGG = Range.clip(speedGG, 0.05, 0.3);
            speedGG = speedGG * Math.signum(errorMaxGG);

            motorGlyphGrab.setPower(speedGG);
            idle();

            curGGPos = motorGlyphGrab.getCurrentPosition();
            if (curGGPos < maxGGPos + 50) // only done with this step if we have reached the target position (maxGGPos) and the motor is not busy
            {
                ggCloseState++;
                driveClose = false; // we're done closing the GG
                motorGlyphGrab.setPower(0.0);
                idle();
            }
        }

        // Glyph Grabber drive to position open
        if(gamepad2.dpad_left)
        {
            driveClose = false;
            ggOpenState = 0;
            driveOpen = true;
            idle();
        }
        else if (driveOpen && ggOpenState == 0)
        {
            motorGlyphGrab.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            curGGPos = motorGlyphGrab.getCurrentPosition();
            telemetry.addData("motorGGCounts:", curGGPos);
            telemetry.update();

            // calculate the speed of the GG motor
            errorMinGG = curGGPos - minGGPos;
            speedGG = Math.abs(errorMinGG * KGlyph);
            speedGG = Range.clip(speedGG, 0.05, 0.25);
            speedGG = speedGG * Math.signum(errorMinGG);
            motorGlyphGrab.setPower(speedGG);
            idle();

            curGGPos = motorGlyphGrab.getCurrentPosition();
            if (curGGPos > minGGPos) // only done with this step if we have reached the target position (maxGGPos) and the motor is not busy
            {
                ggOpenState++;
                driveOpen = false; // we're done closing the GG
                motorGlyphGrab.setPower(0.0);
                idle();
            }
        }

/*
        // calculate the power for each motor (corner drive)
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
/*
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
*/
    public double modJoyStickInput(double i) // i is the raw joystick input
    {
        return Math.pow(i,2) * Math.signum(i);
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