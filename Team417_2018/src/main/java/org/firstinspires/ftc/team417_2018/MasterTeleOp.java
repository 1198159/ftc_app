package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

abstract public class MasterTeleOp extends MasterOpMode
{
    double ly = 0;
    double ry = 0;
    double lx = 0;

    final double ADAGIO_POWER = 0.3;

    boolean isReverseMode = false;
    boolean isLegatoMode = false;

    boolean isServoLowered;
    boolean isXPushed;
    boolean isLeftBumperPushed;
    boolean isRightBumperPushed;

    int curGLPos;
    int tarGLPos;

    AvgFilter filterJoyStickInput = new AvgFilter();

    void tankDrive()
    {
        // hold right trigger for adagio legato mode
        if (gamepad1.right_trigger > 0.0) isLegatoMode = true;
        else isLegatoMode = false;
        // hold left trigger for reverse mode
        if (gamepad1.left_trigger > 0.0) isReverseMode = true;
        else isReverseMode = false;

        if (isLegatoMode) // Legato Mode
        {
            ly = -Range.clip(gamepad1.left_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
            ry = -Range.clip(gamepad1.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
            if (isReverseMode) // Reverse Mode and Legato Mode combo
                ly = Range.clip(gamepad1.left_stick_y, -0.3, 0.3); // Y axis is negative when up
                ry = Range.clip(gamepad1.right_stick_y, -0.3, 0.3); // Y axis is negative when up
        }
        else if (isReverseMode) // Reverse Mode
        {
            ly = gamepad1.left_stick_y; // Y axis is negative when up
            ry = gamepad1.right_stick_y; // Y axis is negative when up
        }
        else // Staccato Mode (standard)
        {
            ly = -gamepad1.left_stick_y; // Y axis is negative when up
            ry = -gamepad1.right_stick_y; // Y axis is negative when up
        }

        filterJoyStickInput.appendInputY(ly, ry);

        ly = filterJoyStickInput.getFilteredLY();
        ry = filterJoyStickInput.getFilteredRY();

        powerFL = ly;
        powerFR = ry;
        powerBL = ly;
        powerBR = ry;
    }

    void arcadeDrive()
    {
        // hold right trigger for adagio legato mode
        if (gamepad1.right_trigger > 0.0) isLegatoMode = true;
        else isLegatoMode = false;
        // hold left trigger for reverse mode
        if (gamepad1.left_trigger > 0.0) isReverseMode = true;
        else isReverseMode = false;

        if (isLegatoMode) // Legato Mode
        {
            ry = -Range.clip(gamepad1.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
            lx = Range.clip(gamepad1.left_stick_x, -ADAGIO_POWER, ADAGIO_POWER);
            if (isReverseMode) // Reverse Mode and Legato Mode combo
                ry = Range.clip(gamepad1.right_stick_y, -0.3, 0.3); // Y axis is negative when up
        }
        else if (isReverseMode) // Reverse Mode
        {
            ry = gamepad1.right_stick_y; // Y axis is negative when up
        }
        else // Staccato Mode (standard)
        {
            ry = -gamepad1.right_stick_y; // Y axis is negative when up
            lx = Range.clip(gamepad1.left_stick_x, -ADAGIO_POWER, ADAGIO_POWER);
        }

        filterJoyStickInput.appendInputY(ly, lx);

        ly = filterJoyStickInput.getFilteredLY();
        lx = filterJoyStickInput.getFilteredRY();

        powerFL = ly - lx;
        powerFR = ry + lx;
        powerBL = ly - lx;
        powerBR = ry + lx;
    }
/*
    void runManualLift()
    {
       // curGLPos = motorLiftLeft.getCurrentPosition();
        // Glyph lift up/down
        if(-gamepad2.right_stick_y < 0.0) // down
        {
            //motorLiftLeft.setPower(Range.clip(-gamepad2.right_stick_y, -0.65, -0.2));
            //motorLiftRight.setPower(Range.clip(-gamepad2.right_stick_y, -0.65, -0.2));
        }
        else if(-gamepad2.right_stick_y > 0.0) // up
        {
            tarGLPos = curGLPos;
            //motorLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //motorLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //motorLiftLeft.setPower(Range.clip(-gamepad2.right_stick_y, 0.2, 0.65));
            //motorLiftRight.setPower(Range.clip(-gamepad2.right_stick_y, 0.2, 0.65));
        }
        else // turn motors off
        {
            //motorLiftLeft.setPower(0.0);
            //motorLiftRight.setPower(0.0);
        }
    }
*/
    void updateTelemetry()
    {
        telemetry.addData("legato: ", isLegatoMode);
        telemetry.addData("reverse: ", isReverseMode);
        telemetry.update();
    }
}
