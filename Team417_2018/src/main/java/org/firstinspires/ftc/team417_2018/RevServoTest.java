package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Rev Servo Test", group="Swerve")
//@Disabled
public class RevServoTest extends LinearOpMode
{
    Servo rev1 = null;
    Servo rev2 = null;
    DcMotor core1 = null; // hub 1 port 3
    DcMotor core2 = null; // hub 2 port 3
    DcMotor arm1 = null;
    DcMotor arm2 = null;
    //CRServo rev2 = null;
    double curExtendPos1 = 0.0;
    double curExtendPos2 = 0.0;

    double rev1pos;
    double rev2pos;

    public void runOpMode()
    {
        // initialize servos
        rev1 = hardwareMap.servo.get("rev1");
        rev2 = hardwareMap.servo.get("rev2");

        core1 = hardwareMap.dcMotor.get("core1");
        core2 = hardwareMap.dcMotor.get("core2");
        arm1 = hardwareMap.dcMotor.get("arm1");
        arm2 = hardwareMap.dcMotor.get("arm2");

        core1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        core2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        core1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        core2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("init", "done");

        telemetry.update();
        rev1.setPosition(0.02);
        rev2.setPosition(0.98);

        waitForStart();

        while (opModeIsActive())
        {
            curExtendPos1 = core1.getCurrentPosition();
            curExtendPos1 = core2.getCurrentPosition();


            // control REV smart servos
            if(gamepad1.dpad_up)
            {
                rev1pos += 0.01;
            }
            else if (gamepad1.dpad_down)
            {
                rev1pos -= 0.01;
            }
            rev1.setPosition(rev1pos);
            rev2.setPosition(rev2pos);

            if(gamepad1.dpad_right)
            {
                rev2pos += 0.01;
            }
            else if (gamepad1.dpad_left)
            {
                rev2pos -= 0.01;
            }

            // control core hex motors
            if (gamepad2.x)
            {
                core1.setPower(gamepad1.right_stick_y);
                core2.setPower(gamepad1.right_stick_y);
            }
            else
            {
                core1.setPower(0.0);
                core2.setPower(0.0);
            }

            // control AM 3.7 motors
            if (gamepad2.x)
            {
                arm1.setPower(gamepad1.left_stick_y);
                arm2.setPower(gamepad1.left_stick_y);
            }
            else
            {
                arm1.setPower(0.0);
                arm2.setPower(0.0);
            }
            telemetry.addData("rev1",rev1pos);
            telemetry.addData("rev2",rev2pos);
        }
    }
}
