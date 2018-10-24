package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Competition")

public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode()
    {
        initHardware();

        waitForStart();

        while (opModeIsActive())
        {
            dankUnderglow(1.0);
            driveMecanumTeleOp();
            runLift();
            telemetry.addData("lift ticks", motorLift.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
