package org.firstinspires.ftc.team8923_2017;

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Move Test", group = "Swerve")
@Disabled
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class MoveTest extends MasterAutonomous
{
    //Declare variables here

    @Override
    public void runOpMode() throws InterruptedException
    {
        //ChooseOptions();

        InitAuto();//Initializes Hardware and sets position based on alliance
        initVuforia();//Initializes Vuforia
        waitForStart();

        double referenceAngle =  imu.getAngularOrientation().firstAngle;
        IMUPivot(referenceAngle, 20, 0.5, 0.015);//Pivot right
        sleep(500);

        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
