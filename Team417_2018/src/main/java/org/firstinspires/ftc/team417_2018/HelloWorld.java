package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Random;

/**
 * This is a simple "hello world" opmode
 *
 */

@Autonomous(name="HelloWorld", group="Swerve")  // @Autonomous(...) is the other common choice
//@Disabled
public class HelloWorld extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        waitForStart();
        land();
        moveTimed(0.2, 200);


    }
}
