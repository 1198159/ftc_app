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

    double core1Power;
    double core2Power;
    final double ADAGIO_POWER = 0.3;

    boolean isReverseMode = false;
    boolean isLegatoMode = false;

    boolean isServoLowered;
    boolean isXPushed;
    boolean isLeftBumperPushed;
    boolean isRightBumperPushed;
    boolean isStraightDrive;

    int arm1pos = 0;


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
        motorBL.setPower(Range.clip(powerBL,-0.6,0.6));
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

    void collector()
    {
        // control core hex motors

        if (gamepad2.right_trigger > 0) // extend the collector
        {
            core1Power = -gamepad2.right_trigger;
            core2Power = -gamepad2.right_trigger;
        }
        else if (gamepad2.left_trigger > 0) // pull the collector in
        {
            core1Power = gamepad2.left_trigger;
            core2Power = gamepad2.left_trigger;
        }
        else
        {
            core1Power = 0.0;
            core2Power = 0.0;
        }
        core1.setPower(core1Power);
        core2.setPower(core2Power);


        //rev1.setPosition(Range.clip(-gamepad2.left_stick_y, -1.0, 1.0));
        rev1.setPosition(gamepad2.left_stick_y);

        // control AM 3.7 motors
        if (gamepad2.right_stick_y != 0)
        {
            arm1.setPower(Range.clip(gamepad2.right_stick_y, -0.15, 0.15));
            arm2.setPower(Range.clip(-gamepad2.right_stick_y, -0.15, 0.15));
        }
        else
        {
            arm1.setPower(0.0);
            arm2.setPower(0.0);
        }
        arm1pos = arm1.getCurrentPosition(); // update arm1 motor position

        // control hanger
        if (gamepad2.left_bumper)
        {
            hanger.setPower(0.99);
        }
        else if (gamepad2.right_bumper)
        {
            hanger.setPower(-0.99);
        }
        else
        {
            hanger.setPower(0.0);
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
        telemetry.addData("arm1:", arm1pos);
        telemetry.update();
    }
}
