package org.firstinspires.ftc.team8923;

/**
 * This class contains code for running the CapBot with all driver controls
 */
public class TeleOpCapBotCompetition extends MasterTeleOpCapBot
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();

        waitForStart();

        while(opModeIsActive())
        {
            mecanumDrive();

            sendTelemetry();
            idle();
        }
    }
}
