package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Program to test functionality of encoders
 */

@Autonomous(name = "Encoder Test")
public class EncoderTest extends MasterOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {
            motorBackRight.setPower(1.0);
            motorBackLeft.setPower(1.0);
            motorFrontRight.setPower(1.0);
            motorFrontLeft.setPower(1.0);

            telemetry.addData("BR enc value ", motorBackRight.getCurrentPosition());
            telemetry.addData("BL enc value ", motorBackLeft.getCurrentPosition());
            telemetry.addData("FR enc value ", motorFrontRight.getCurrentPosition());
            telemetry.addData("FL enc value ", motorFrontLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}
