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

        DropJJ();
        sleep(2000);
        RetreiveJJ();
        sleep(1000);
        MoveIMU(100, 0.0, 0.035, 0.6, 1);

        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
