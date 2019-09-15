package org.firstinspires.ftc.team6220_2019;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        while(opModeIsActive())
        {
            waitForStart();

            driveMecanumWithJoysticks();

            idle();
        }
    }
}
