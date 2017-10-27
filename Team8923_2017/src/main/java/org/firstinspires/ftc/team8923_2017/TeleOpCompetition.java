package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.GenericSignatureFormatError;

/**
 * Runable shell for Master TeleOp code
 */

@TeleOp(name = "CapBot Teleop")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        InitHardware();

        waitForStart();

        while (opModeIsActive())
        {
            DriveOmni45TeleOp();
            RunGG();
            SendTelemetry();
            //idle();
        }
    }
}

