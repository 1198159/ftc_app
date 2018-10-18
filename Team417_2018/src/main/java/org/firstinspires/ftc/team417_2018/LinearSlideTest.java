package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Linear Slide Test")
public class LinearSlideTest extends MasterTeleOp
{
    public void runOpMode() throws InterruptedException
    {
        super.initializeHardware();

        waitForStart();
        while (opModeIsActive())
        {
           // runManualLift();
            telemetry.addData("hi","there");
            telemetry.update();
        }
    }
}
