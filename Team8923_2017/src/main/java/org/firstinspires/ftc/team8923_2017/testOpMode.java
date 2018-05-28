package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * A test to confirm that we can compile code in each new team project
 */
@Disabled
@TeleOp(name="test 8923_2017", group = "Swerve")
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
