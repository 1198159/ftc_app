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

    boolean isReverseMode = false;
    boolean isLegatoMode = false;
    boolean isCornerDrive = false;
    boolean driveClose = false;
    boolean driveOpen = false;

    int ggOpenState;
    int ggCloseState;
    int liftState; // keeps track of the state that GL is in (either -1, 0, 1)
    int liftLevel = 0; // is either 0, 1, 2, or 3
    int glyphCounts = 1150; // this is the height of a glyph in encoder count values
    boolean isLifting = false;
    boolean isLowering = false;


    AvgFilter filterJoyStickInput = new AvgFilter();

    // declare variables for the GG (AndyMarkNeveRest 3.7 motor is 44.4 counts per rev)
    double KGlyph = 1.0f/600.0f; // constant for proportional drive
    double MINSPEED = 0.05;
    double MAXSPEED = 0.4;

    double speedGG; // current speed
    int errorMaxGG; // error for closing grabber
    int errorMinGG; // error for opening grabber
    int prevGGError = 0; // error of GG motor in previous loop

    int curGGPos;
    int curGLPos;
    int tarGLPos;
    int tolGL = 20;
    int minGGPos = -180; // a bit less than the original starting position of zero (where we start it) (OPEN is more positive)
    int maxGGPos = -577; // maxGGPos equals the # rev to close/open GG (13 rev) times 44.4 counts per rev (CLOSED is more negative)
    int maxGLPos = 4000;

    void omniDriveTeleOp()
    {
        // hold right trigger for adagio legato mode
        if (gamepad1.right_trigger > 0.0 && !isCornerDrive)
            isLegatoMode = true;
        else
            isLegatoMode = false;

        // hold left trigger for reverse mode
        if (gamepad1.left_trigger > 0.0 && !isLegatoMode)
            isReverseMode = false;
        else
            isReverseMode = true;

        if (isLegatoMode) // Legato Mode
        {
            y = -Range.clip(gamepad1.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
            x = Range.clip(gamepad1.right_stick_x, -ADAGIO_POWER, ADAGIO_POWER);
            if (gamepad1.dpad_left) x = -0.3;
            if (gamepad1.dpad_right) x = 0.3;
            if (gamepad1.dpad_down) y = -0.3;
            if (gamepad1.dpad_up) y = 0.3;
            //pivotPower = Range.clip(gamepad1.left_stick_x, -0.3, 0.3);
            pivotPower = (gamepad1.left_stick_x) * 0.3;
        }
        else // Staccato Mode
        {
            y = -gamepad1.right_stick_y; // Y axis is negative when up
            x = gamepad1.right_stick_x;
            if (gamepad1.dpad_left) x = -0.75;
            if (gamepad1.dpad_right) x = 0.75;
            if (gamepad1.dpad_down) y = -0.75;
            if (gamepad1.dpad_up) y = 0.75;
            //pivotPower = Range.clip(gamepad1.left_stick_x, -0.9, 0.9);
            pivotPower = (gamepad1.left_stick_x) * 0.9;
        }

        filterJoyStickInput.appendInput(x, y, pivotPower);
        x = filterJoyStickInput.getFilteredX() + ((curGGPos-prevGGError)/-40);
        prevGGError = motorGlyphGrab.getCurrentPosition();
        y = filterJoyStickInput.getFilteredY();
        pivotPower = filterJoyStickInput.getFilteredP();

        if (isCornerDrive)
        {
            // corner drive
            powerFL = -y + pivotPower;
            powerFR = x - pivotPower;
            powerBL = x + pivotPower;
            powerBR = -y - pivotPower;
        }
        else if (isReverseMode)
        {
            // calculate the power for each motor (this is the default 'forward' in TeleOp)
            powerFL = -x - y + pivotPower;
            powerFR = x - y - pivotPower;
            powerBL = x - y + pivotPower;
            powerBR = -x - y - pivotPower;
        }
        else
        {
            powerFL = x + y + pivotPower;
            powerFR = -x + y - pivotPower;
            powerBL = -x + y + pivotPower;
            powerBR = x + y - pivotPower;
        }
        setDriveMotorPower();
    }


    void runJJ()
    {
        // JJ SOFTWARE (hold GamePad2 button "B" down for JJ Low Position
        servoJewel.setPosition(Range.clip((1-gamepad2.right_trigger), JEWEL_LOW, JEWEL_INIT));
        //if (gamepad2.b) servoJewel.setPosition(JEWEL_LOW + 0.07);
        //else servoJewel.setPosition(JEWEL_INIT);
    }


    void runManualGlyphLift()
    {
        curGLPos = motorGlyphLeft.getCurrentPosition();
        // Glyph lift up/down
        if(-gamepad2.right_stick_y < 0.0 && !isLifting) // down
        {
            tarGLPos = curGLPos;
            motorGlyphLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorGlyphRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorGlyphLeft.setPower(Range.clip(-gamepad2.right_stick_y, -0.65, -0.2));
            motorGlyphRight.setPower(Range.clip(-gamepad2.right_stick_y, -0.65, -0.2));
        }
        else if(-gamepad2.right_stick_y > 0.0 && curGLPos<maxGLPos && !isLifting) // up
        {
            tarGLPos = curGLPos;
            motorGlyphLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorGlyphRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorGlyphLeft.setPower(Range.clip(-gamepad2.right_stick_y, 0.2, 0.65));
            motorGlyphRight.setPower(Range.clip(-gamepad2.right_stick_y, 0.2, 0.65));
        }
        else // turn motors off
        {
            if (!isLifting)
            {
                motorGlyphLeft.setPower(0.0);
                motorGlyphRight.setPower(0.0);
            }
        }
    }

    void runAutoGlyphLift()
    {
        if (gamepad2.y) // LIFT ONE GLYPH LEVEL
        {
            isLifting = true;
            liftState = 1;
            motorGlyphLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorGlyphRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tarGLPos = Range.clip( ((liftLevel + 1) * glyphCounts) , 0, maxGLPos);
            idle();
        }
        else if (isLifting && liftState==1)
        {
            // lift the GM
            curGLPos = motorGlyphLeft.getCurrentPosition();
            motorGlyphLeft.setTargetPosition(tarGLPos);
            motorGlyphRight.setTargetPosition(tarGLPos);

            motorGlyphLeft.setPower(0.65);
            motorGlyphRight.setPower(0.65);

            if (curGLPos > maxGLPos || curGLPos > tarGLPos-tolGL) // stop conditions
            {
                motorGlyphLeft.setPower(0.0);
                motorGlyphRight.setPower(0.0);
                liftState = 0; // not lifting anymore
                isLifting = false;
                motorGlyphLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorGlyphRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                idle();
            }
        }

        if (gamepad2.a) // LIFT ONE GLYPH LEVEL
        {
            isLifting = true;
            liftState = -1;
            motorGlyphLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorGlyphRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tarGLPos = Range.clip( ((liftLevel - 1) * glyphCounts) , 0, maxGLPos);
            idle();
        }
        else if (isLifting && liftState==-1)
        {
            // lift the GM
            curGLPos = motorGlyphLeft.getCurrentPosition();
            motorGlyphLeft.setTargetPosition(tarGLPos);
            motorGlyphRight.setTargetPosition(tarGLPos);

            motorGlyphLeft.setPower(-0.65);
            motorGlyphRight.setPower(-0.65);

            if (curGLPos < 0 || curGLPos < tarGLPos+tolGL) // stop conditions
            {
                motorGlyphLeft.setPower(0.0);
                motorGlyphRight.setPower(0.0);
                liftState = 0; // not lifting anymore
                isLifting = false;
                motorGlyphLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorGlyphRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                idle();
            }
        }

        liftLevel = (int) Math.floor((motorGlyphLeft.getCurrentPosition()+tolGL) / glyphCounts);
    }


    void runManualGG()
    {
        // Manual Glyph grabber open/close
        if (!driveOpen && !driveClose)
            curGGPos = motorGlyphGrab.getCurrentPosition(); // set the current position of the GG
        if(gamepad2.left_bumper && curGGPos > maxGGPos) // CLOSE (counter goes negative when closing)
        {
            motorGlyphGrab.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            errorMaxGG = curGGPos - maxGGPos;
            speedGG = Math.abs(errorMaxGG * KGlyph);
            speedGG = Range.clip(speedGG, MINSPEED, MAXSPEED);
            speedGG = speedGG * Math.signum(errorMaxGG);
            motorGlyphGrab.setPower(speedGG);
        }
        else if(gamepad2.right_bumper && curGGPos < minGGPos) // OPEN (counter goes positive when opening)
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
    }




    void runAutoGG()
    {
        // CLOSE
        if(gamepad2.dpad_left)
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

        // OPEN
        if(gamepad2.dpad_right)
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

    void updateTelemetry()
    {
        //telemetry.addData("legato: ", isLegatoMode);
        //telemetry.addData("reverse: ", isReverseMode);
        telemetry.addData("GlyphLeftPos: ", motorGlyphLeft.getCurrentPosition());
        telemetry.addData("GlyphRightPos: ", motorGlyphRight.getCurrentPosition());
        telemetry.addData("LiftLevel: ", liftLevel);
        telemetry.addData("tarGLPos: ", tarGLPos);
        //telemetry.addData("GlyphGrabPos: ", motorGlyphGrab.getCurrentPosition());
        telemetry.update();
    }

    public double modJoyStickInput(double i) // i is the raw joystick input
    {
        return Math.pow(i,2) * Math.signum(i);
    }

    public void setDriveMotorPower()
    {
        // set power to the motors
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
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