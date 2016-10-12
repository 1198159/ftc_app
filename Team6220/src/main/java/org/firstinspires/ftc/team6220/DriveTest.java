package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
    Test op mode that drives the robot with motor powers.
*/
@TeleOp(name="Test Drive", group="Tests")
@Disabled
public class DriveTest extends MasterTeleOp
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {

            /*;
            ;
            ;*/

            //@TODO:  encapsulate stick curve in MasterTeleOp
            drive.moveRobot(-1.0 * (0.5 * Math.pow(gamepad1.right_stick_x , 3) + 0.5 * gamepad1.right_stick_x),   //local x motion power
                            -1.0 * (0.5 * Math.pow(gamepad1.right_stick_y , 3) + 0.5 * gamepad1.right_stick_y),   //local y motion power
                            -1.0 * (0.5 * Math.pow(gamepad1.left_stick_x , 3) + 0.5 * gamepad1.left_stick_x)); //rotation power; Rotation is reversed

            idle();
        }
    }
}
