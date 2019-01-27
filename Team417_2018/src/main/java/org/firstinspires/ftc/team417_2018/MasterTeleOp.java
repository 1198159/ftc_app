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

    double curRevPos = INIT_REV_POS; // starts in down position

    int core2Position;
    double core2Power;
    int core1Pos;
    int core2Pos;
    final double ADAGIO_POWER = 0.3;

    boolean isReverseMode = false;
    boolean isLegatoMode = false;

    boolean isServoLowered;
    boolean isXPushed;

    boolean isLeftBumperPushed = false;
    boolean isSpittingOut = false;
    boolean isRightBumperPushed = false;
    boolean isSuckingIn = false;


    boolean isDpadLeftPushed = false;
    boolean isCollectorCenter = false;

    boolean isDpadUpPushed = false;
    boolean isCollectorUp = false;

    boolean isDpadDownPushed = false;
    boolean isCollectorDown = false; // gamepad joystick up

    boolean isMarkerDown = true;
    boolean isYButtonPressed = true;

    boolean isStraightDrive;


    int arm1pos = 0;


    int tarGLPos;

    AvgFilter filterJoyStickInput = new AvgFilter();


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
            if (gamepad1.dpad_left) x = -0.2;
            if (gamepad1.dpad_right) x = 0.2;
            if (gamepad1.dpad_down) y = -0.2;
            if (gamepad1.dpad_up) y = 0.2;
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
            if (gamepad1.right_stick_y != 0) y = gamepad1.right_stick_y;
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

    void collector()
    {

// control the extending with G2 right (extend) and left (retract) trigger
        if (gamepad2.right_trigger != 0) core2.setPower(gamepad2.right_trigger);
        else if (gamepad2.left_trigger != 0) core2.setPower(-gamepad2.left_trigger);
        else core2.setPower(0.0);

// control arm motors with G2 right stick
        if (gamepad2.right_stick_y != 0)
        {
            arm1.setPower(Range.clip(gamepad2.right_stick_y, -0.3, 0.3));
            arm2.setPower(Range.clip(-gamepad2.right_stick_y, -0.3, 0.3));
        }
        else
        {
            arm1.setPower(0.0);
            arm2.setPower(0.0);
        }
        arm1pos = arm1.getCurrentPosition(); // update arm1 motor position

// control hanger with G2 left and right bumpers
        if (gamepad2.dpad_up)
        {
            hanger.setPower(0.99); // extend the hanger
        }
        else if (gamepad2.dpad_down)
        {
            hanger.setPower(-0.99); // retract hanger
        }
        else
        {
            hanger.setPower(0.0);
        }

// control rev servo with G2 left stick
        if (-gamepad2.left_stick_y > 0.1 && curRevPos > 0.0) // if the joystick is UP
        {
            curRevPos = curRevPos - REV_INCREMENT; // move the collector up
        }
        else if (-gamepad2.left_stick_y < -0.1 && curRevPos < 0.9) // if the joystick is DOWN
        {
            curRevPos = curRevPos + REV_INCREMENT; // move the collector down
        }
        rev1.setPosition(curRevPos); // set the wrist REV servo position


        if (gamepad2.dpad_left && !isDpadLeftPushed)
        {
            isDpadLeftPushed = true;
            isCollectorCenter = !isCollectorCenter;
        }
        isDpadLeftPushed = gamepad2.dpad_left;
        if (isCollectorCenter)
        {
            rev1.setPosition(0.5);
            isCollectorDown = false;
            isCollectorCenter = false;
        }

        telemetry.addData("legato: ", isLegatoMode);
        telemetry.update();
        idle();

        // control vex Servo
        //boolean isRightBumperPushed = false;
        //boolean isSuckingIn = false;

        // we include is right bumper pushed because this code is an event loop and it is constantly running that means that when
        //  a person pushes the button once it could run through the loop almost 50 times and update the state inaccurately in a true-false-true-
        // false way that could leave left_bumper on an incorrect state. By adding isRightBumperPushed we make the state change the first time around
        // so that it won't register more than once and the condition for changing state will remain false after one run through

        if (gamepad2.left_bumper)
        {
            isSuckingIn = false; // cancel sucking in
            vex1.setPower(0.79); // if you press the left bumper release minerals
        }
        if (!gamepad2.left_bumper && !isSuckingIn) vex1.setPower(0.0); // if you are not pressing the left bumper do not set power to the vex motor

        if(isSuckingIn) vex1.setPower(-0.79); // if sucking in is true then set negative power so the servo spins opposite way

        if (gamepad2.right_bumper && !isRightBumperPushed) // if the right bumper is pressed && boolean for isRightBumperPushed true( that means it is currently false)
        {
            isRightBumperPushed = true; // switch the value of right bumper pushed to true
            isSuckingIn = !isSuckingIn; // and switch sucking in's boolean to sucking out or in depending on what it is
        }
        isRightBumperPushed = gamepad2.right_bumper; // update the current state of isRightBumperPushed otherwise it will always stay the sa,e
    }

    void marker()
    {
        // Press button y to toggle up and down
        if(isMarkerDown) marker.setPosition(MARKER_LOW);
        else marker.setPosition(MARKER_HIGH);

        if (gamepad2.y && !isYButtonPressed)
        {
            isYButtonPressed = true;
            isMarkerDown = !isMarkerDown;
        }
        isYButtonPressed = gamepad2.y;
    }

    void updateTelemetry()
    {
        telemetry.addData("legato: ", isLegatoMode);
        telemetry.addData("reverse: ", isReverseMode);
        telemetry.addData("rev1:", rev1.getPosition());
        telemetry.addData("core2:", core2Pos);
        telemetry.addData("hanger:", hanger.getCurrentPosition());
        telemetry.update();
    }
}
