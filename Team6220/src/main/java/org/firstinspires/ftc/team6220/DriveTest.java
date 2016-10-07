package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    Test op mode that drives the robot with motor powers.
*/
@TeleOp(name="Test Drive", group="Tests")
public class DriveTest extends MasterTeleOp
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {
            drive.moveRobot(powerRightStickX,   //local x motion power
                            powerRightStickY,   //local y motion power
                            -1.0 * powerLeftStickX); //rotation power; Rotation is reversed

            idle();
        }
    }
}
