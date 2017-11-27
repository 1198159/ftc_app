package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Move Test 2", group = "Swerve")
@Disabled
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class MoveTest2 extends MasterAutonomous
{
    //Declare variables here

    @Override
    public void runOpMode() throws InterruptedException
    {
        //ChooseOptions();

        InitAuto();//Initializes Hardware and sets position based on alliance
        initVuforia();//Initializes Vuforia

        waitForStart();

        alignOnLine55(0.5);

        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
