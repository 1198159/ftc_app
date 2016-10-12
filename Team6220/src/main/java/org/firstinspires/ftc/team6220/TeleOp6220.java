package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Colew on 9/18/2016.
 */
@TeleOp(name="TeleOp", group="6220")
public class TeleOp6220 extends MasterTeleOp
{
    public void runOpMode() throws InterruptedException
    {
        //initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {
            driveRobotWithJoysticks();

            idle();
        }
    }
}