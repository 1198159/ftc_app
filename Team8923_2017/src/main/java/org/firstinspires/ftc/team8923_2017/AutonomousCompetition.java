package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Autonomous Competition", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutonomousCompetition extends MasterAutonomous
{
    //Declare variables here

    @Override
    public void runOpMode() throws InterruptedException
    {
        //ChooseOptions();

        InitAuto();//Initializes Hardware and sets position based on alliance

        waitForStart();

        //GetLeftJewelColor();
        //if (IsLeftJewelRed)
        //DropJJ();
        //sleep(1000);
        //pivot(30, 0.3, 1);//moves 10 MM, speed, timeout
        //sleep(1000);
        //pivot(40, 0.2, 0.5);
        IMUPivot(90, 0.5, 0.25);
         while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
