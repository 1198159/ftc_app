package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Competition Red Pos 2", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutonomousCompetitionRedPos2 extends MasterAutonomous
{
    //Declare variables here

    @Override
    public void runOpMode() throws InterruptedException
    {
        //ChooseOptions();

        InitAuto();//Initializes Hardware and sets position based on alliance
        initVuforia();//Initializes Vuforia

        waitForStart();

        DropJJ();
        sleep(1000);
        GetLeftJewelColor();
        sleep(1000);
        double referenceAngle =  imu.getAngularOrientation().firstAngle;

        if (isLeftJewelRed = true)
        {
            IMUPivot(referenceAngle, -20, 0.5, 0.001);
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.001);
        }
        else
        {
            IMUPivot(referenceAngle, 20, 0.5, 0.001);
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.001);
        }

        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
