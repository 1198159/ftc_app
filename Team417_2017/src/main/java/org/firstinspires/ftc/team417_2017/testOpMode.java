package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;


/**
 * A test to confirm that we can compile code in each new team project
 */
@TeleOp(name="test 417_2017", group = "Swerve")
// @Disabled
public class testOpMode extends LinearOpMode
{

    @Override public void runOpMode() throws InterruptedException
    {
        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while(opModeIsActive())
        {
            idle();
        }
    }

}
