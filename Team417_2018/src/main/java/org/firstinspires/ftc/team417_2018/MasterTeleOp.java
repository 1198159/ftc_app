package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

abstract public class MasterTeleOp extends MasterOpMode
{
    double y = 0;
    double x = 0;
    double pivotPower = 0;
    final double ADAGIO_POWER = 0.3;

    boolean isReverseMode = false;
    boolean isLegatoMode = false;
    boolean isStraightDrive = false;
    boolean driveClose = false;
    boolean driveOpen = false;

    boolean isServoLowered;
    boolean isXPushed;
    boolean isLeftBumperPushed;
    boolean isRightBumperPushed;

    int ggOpenState;
    int ggCloseState;
    int liftState; // keeps track of the state that GL is in (either -1, 0, 1)
    int liftLevel = 0; // is either 0, 1, 2, or 3
    int glyphCounts = 1300; // this is the height of a glyph in encoder count values
    boolean isLifting = false;
    boolean isLowering = false;

    // declare variables for the GG (AndyMarkNeveRest 3.7 motor is 44.4 counts per rev)
    double KGlyph = 1.0f/600.0f; // constant for proportional drive
    double MINSPEED = 0.05;
    double MAXSPEED = 0.4;

    double speedGG; // current speed
    int errorMaxGG; // error for closing grabber
    int errorMinGG; // error for opening grabber
    int prevGGError = 0; // error of GG motor in previous loop
    double shiftGG = 0.0;

    int curGLPos;
    int tarGLPos;
    int tolGL = 20;
    double speedGL;
    int curGGPos;
    int minGGPos = -180; // a bit less than the original starting position of zero (where we start it) (OPEN is more positive)
    int maxGGPos = -577; // maxGGPos equals the # rev to close/open GG (13 rev) times 44.4 counts per rev (CLOSED is more negative)
    int maxGLPos = 4000;

    void runManualLift()
    {
        curGLPos = motorLiftLeft.getCurrentPosition();
        // Glyph lift up/down
        if(-gamepad2.right_stick_y < 0.0) // down
        {
            motorLiftLeft.setPower(Range.clip(-gamepad2.right_stick_y, -0.65, -0.2));
            motorLiftRight.setPower(Range.clip(-gamepad2.right_stick_y, -0.65, -0.2));
        }
        else if(-gamepad2.right_stick_y > 0.0) // up
        {
            tarGLPos = curGLPos;
            motorLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLiftLeft.setPower(Range.clip(-gamepad2.right_stick_y, 0.2, 0.65));
            motorLiftRight.setPower(Range.clip(-gamepad2.right_stick_y, 0.2, 0.65));
        }
        else // turn motors off
        {
            motorLiftLeft.setPower(0.0);
            motorLiftRight.setPower(0.0);
        }
    }
}
