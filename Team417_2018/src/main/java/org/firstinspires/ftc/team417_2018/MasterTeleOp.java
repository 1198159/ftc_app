package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

abstract public class MasterTeleOp extends MasterOpMode
{
    double ly = 0;
    double ry = 0;
    double lx = 0;
    double x = 0;
    double y = 0;
    double pivotPower = 0;

    final double ADAGIO_POWER = 0.3;

    boolean isReverseMode = false;
    boolean isLegatoMode = false;

    boolean isServoLowered;
    boolean isXPushed;
    boolean isLeftBumperPushed;
    boolean isRightBumperPushed;
    boolean isStraightDrive;

    int curLiftPos;
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
                ry = -Range.clip(gamepad1.right_stick_y, -0.3, 0.3); // Y axis is negative when up
        }
        else if (isReverseMode) // Reverse Mode
        {
            ry = gamepad1.left_stick_y; // Y axis is negative when up
            ly = gamepad1.right_stick_y; // Y axis is negative when up
        }
        else // Staccato Mode (standard)
        {
            ly = -gamepad1.left_stick_y; // Y axis is negative when up
            ry = -gamepad1.right_stick_y; // Y axis is negative when up
        }

        if (gamepad1.dpad_down)
        {
            ly = -0.3;
            ry = -0.3;
        }
        if (gamepad1.dpad_up)
        {
            ly = 0.3;
            ry = 0.3;
        }

        filterJoyStickInput.appendInputY(ly, ry);

        ly = filterJoyStickInput.getFilteredLY();
        ry = filterJoyStickInput.getFilteredRY();

        powerFL = ly;
        powerFR = ry;
        powerBL = ly;
        powerBR = ry;

        motorFL.setPower(powerFL);
        motorBL.setPower(powerBL);
        motorFR.setPower(powerFR);
        motorBR.setPower(powerBR);
    }

    void mecanumDrive()
    {
        // hold right bumper for adagio legato mode
        if (gamepad1.right_trigger>0) isLegatoMode = true;
        else isLegatoMode = false;
        // hold left bumper for reverse mode
        if (gamepad1.left_trigger>0) isReverseMode = true;
        else isReverseMode = false;


        if (isLegatoMode) // Legato Mode
        {
            y = -Range.clip(-gamepad1.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
            x = -Range.clip(gamepad1.right_stick_x, -ADAGIO_POWER, ADAGIO_POWER);
            if (gamepad1.dpad_left) x = -0.3;
            if (gamepad1.dpad_right) x = 0.3;
            if (gamepad1.dpad_down) y = -0.3;
            if (gamepad1.dpad_up) y = 0.3;
            pivotPower = Range.clip(gamepad1.left_stick_x, -0.3, 0.3);

            if (isReverseMode) // if both legato and reverse mode
            {
                y = Range.clip(-gamepad1.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
                x = Range.clip(gamepad1.right_stick_x, -ADAGIO_POWER, ADAGIO_POWER);
                if (gamepad1.dpad_left) x = -0.3;
                if (gamepad1.dpad_right) x = 0.3;
                if (gamepad1.dpad_down) y = -0.3;
                if (gamepad1.dpad_up) y = 0.3;
                pivotPower = Range.clip(gamepad1.left_stick_x, -0.3, 0.3);
            }
        }
        else if (isReverseMode)
        {
            y = Range.clip(-gamepad1.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER); // Y axis is negative when up
            x = Range.clip(gamepad1.right_stick_x, -ADAGIO_POWER, ADAGIO_POWER);
            if (gamepad1.dpad_left) x = -0.75;
            if (gamepad1.dpad_right) x = 0.75;
            if (gamepad1.dpad_down) y = -0.75;
            if (gamepad1.dpad_up) y = 0.75;
            pivotPower = Range.clip(gamepad1.left_stick_x, -0.3, 0.3);
        }
        else // Staccato Mode
        {
            y = gamepad1.right_stick_y; // Y axis is negative when up
            x = -gamepad1.right_stick_x;
            if (gamepad1.dpad_left) x = -0.75;
            if (gamepad1.dpad_right) x = 0.75;
            if (gamepad1.dpad_down) y = -0.75;
            if (gamepad1.dpad_up) y = 0.75;
            //pivotPower = Range.clip(gamepad1.left_stick_x, -0.9, 0.9);
            pivotPower = (gamepad1.left_stick_x) * 0.95;
        }

        filterJoyStickInput.appendInput(x, y, pivotPower);

        x = filterJoyStickInput.getFilteredX();
        y = filterJoyStickInput.getFilteredY();
        pivotPower = filterJoyStickInput.getFilteredP();

        powerFL = -x - y + pivotPower;
        powerFR = x - y - pivotPower;
        powerBL = x - y + pivotPower;
        powerBR = -x - y - pivotPower;

        motorFL.setPower(powerFL);
        motorBL.setPower(powerBL);
        motorFR.setPower(powerFR);
        motorBR.setPower(powerBR);
    }

    void simpleDrive()
    {
        ly = -gamepad1.left_stick_y; // Y axis is negative when up
        ry = -gamepad1.right_stick_y; // Y axis is negative when up

        powerFL = ly;
        powerFR = ry;
        powerBL = ly;
        powerBR = ry;

        motorFL.setPower(powerFL);
        motorBL.setPower(powerBL);
        motorFR.setPower(powerFR);
        motorBR.setPower(powerBR);
    }

    void runManualLift()
    {
         curLiftPos = motorLift.getCurrentPosition();
        // Glyph lift up/down
        if(/*-gamepad2.right_stick_y < 0.0*/ gamepad1.left_bumper) // down
        {
            //motorLift.setPower(Range.clip(-gamepad2.right_stick_y, -1.0, -0.2));
            motorLift.setPower(-1.0);
        }
        else if(/*-gamepad2.right_stick_y > 0.0*/gamepad1.right_bumper) // up
        {
            //motorLift.setPower(Range.clip(-gamepad2.right_stick_y, 0.2, 1.0));
            motorLift.setPower(1.0);
        }
        else // turn motors off
        {
            motorLift.setPower(0.0);
        }
    }

    void marker()
    {
        marker.setPosition(Range.clip((gamepad2.right_trigger), MARKER_LOW, MARKER_HIGH));
    }

    void updateTelemetry()
    {
        telemetry.addData("legato: ", isLegatoMode);
        telemetry.addData("reverse: ", isReverseMode);
        telemetry.addData("lift motor", curLiftPos);
        telemetry.update();
    }
}
