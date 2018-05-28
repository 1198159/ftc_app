package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by coles on 11/21/2017.
 */

@TeleOp(name = "Encoder Test", group = "Test")
@Disabled
public class EncoderFunctionalityTest extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        InitHardware();

        waitForStart();

        while (opModeIsActive())
        {
            DriveOmni45TeleOp();
            SendTelemetry();
            //idle();
        }
    }

    @Override
    public void SendTelemetry()
    {
        telemetry.addData("Encoder FL", motorFL.getCurrentPosition());
        telemetry.addData("Encoder FR", motorFR.getCurrentPosition());
        telemetry.addData("Encoder BL", motorBL.getCurrentPosition());
        telemetry.addData("Encoder BR", motorBR.getCurrentPosition());
        telemetry.update();
    }
}
