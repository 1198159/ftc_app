package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.org.apache.xml.internal.security.Init;

@TeleOp(name = "TeleOp Competition")

class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode()
    {
        InitHardware();

        waitForStart();

        while (opModeIsActive())
        {
            driveMecanumTeleOp();
        }
    }
}
