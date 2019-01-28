package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Linear Slide Test")
//@Disabled
public class LinearSlideTest extends MasterTeleOp
{
    int targetPos = 0;

    public void runOpMode() throws InterruptedException
    {
        super.initializeHardware();
        core2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set encoder value to 0
        core2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Init","done");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            if (gamepad2.right_trigger != 0 && targetPos < MAX_CORE_POS)
            {
                targetPos++;
                core2.setTargetPosition(targetPos);
                core2.setPower(0.7);
            }
            if (gamepad2.left_trigger != 0 && targetPos > -50)
            {
                targetPos--;
                core2.setTargetPosition(targetPos);
                core2.setPower(0.8);
            }

            telemetry.addData("tarPos", targetPos);
            telemetry.update();
        }
    }
}
