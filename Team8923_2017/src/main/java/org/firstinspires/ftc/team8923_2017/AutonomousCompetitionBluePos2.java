package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Autonomous Competition Blue Pos 2", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutonomousCompetitionBluePos2 extends MasterAutonomous
{
    //Declare variables here

    @Override
    public void runOpMode() throws InterruptedException
    {
        //ChooseOptions();

        InitAuto();//Initializes Hardware and sets position based on alliance
        initVuforia();//Initializes Vuforia

        waitForStart();

        closeGG();
        sleep(500);
        moveGG(1500);
        DropJJ();
        sleep(1000);
        GetVumark();
        GetLeftJewelColor();
        double referenceAngle =  imu.getAngularOrientation().firstAngle;

        if (isLeftJewelRed == true)
        {
            IMUPivot(referenceAngle, 20, 0.5, 0.015);//Pivot right
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);//Pivot left
        }
        else
        {
            IMUPivot(referenceAngle, -20, 0.5, 0.015);
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);
        }

        MoveIMU(referenceAngle, 40.0, 0.0, 0.015, 0.5, 2.5);//Go towards parking spot
        IMUPivot(referenceAngle, -90, 0.5, 0.015);
        referenceAngle -= 90.0;
        referenceAngle = adjustAngles(referenceAngle);
        moveGG(-1500);

        //if loo[ here
        if (vuMark == RelicRecoveryVuMark.LEFT)
        {

            MoveIMU(referenceAngle, 100.0, 0.0, 0.015, 0.3, 1.9);
            IMUPivot(referenceAngle, 90, 0.5, 0.015);
            referenceAngle += 90.0;
            referenceAngle = adjustAngles(referenceAngle);
            MoveIMU(referenceAngle, 70.0, 0.0, 0.015, 0.3, 1.0);
        }
        if (vuMark == RelicRecoveryVuMark.CENTER)
        {
            //WORKS!
            MoveIMU(referenceAngle, 100.0, 0.0, 0.015, 0.5, 0.8);
            IMUPivot(referenceAngle, 90, 0.5, 0.015);
            referenceAngle += 90.0;
            referenceAngle = adjustAngles(referenceAngle);
            MoveIMU(referenceAngle, 70.0, 0.0, 0.015, 0.3, 1.0);
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT);
        {
            //WORKS!
            MoveIMU(referenceAngle, 80.0, 0.0, 0.015, 0.5, 0.5);
            IMUPivot(referenceAngle, 90, 0.5, 0.015);
            referenceAngle += 90.0;
            referenceAngle = adjustAngles(referenceAngle);
            MoveIMU(referenceAngle, 70.0, 0.0, 0.015, 0.3, 1.0);
        }
        openGG();
        MoveIMU(referenceAngle, -50.0, 0.0, 0.015, 0.5, 0.3);//Go towards parking spot


        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}

