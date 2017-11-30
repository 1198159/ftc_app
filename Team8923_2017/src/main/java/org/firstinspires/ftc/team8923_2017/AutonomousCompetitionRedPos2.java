package org.firstinspires.ftc.team8923_2017;

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

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
        GetVumark();

        waitForStart();
        // turn on flash light
        CameraDevice.getInstance().setFlashTorchMode(true);
        // set false to turn off light
        closeGG();
        sleep(500);
        moveGG(1500);
        DropJJ();
        sleep(1000);
        //turnOnFlash(4000);
        GetLeftJewelColor();
        CameraDevice.getInstance().setFlashTorchMode(false);
        double referenceAngle =  imu.getAngularOrientation().firstAngle;

        if (isLeftJewelRed == true)
        {
            IMUPivot(referenceAngle, 10, 0.5, 0.015);//Pivot right
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);//Pivot left
        }
        else
        {
            IMUPivot(referenceAngle, -10, 0.5, 0.015);
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);
        }

        MoveIMU(referenceAngle, -400.0, 0.0, 0.015, 0.3, 1.7);//Go towards parking spot
        IMUPivot(referenceAngle, -90, 0.25, 0.015);
        referenceAngle -= 90.0;
        referenceAngle = adjustAngles(referenceAngle);

        MoveIMU(referenceAngle, 100.0, 0.0, 0.015, 0.3, 1.2);
        IMUPivot(referenceAngle, -88, 0.25, 0.015);
        referenceAngle -= 90.0;
        referenceAngle = adjustAngles(referenceAngle);

        //if loo[ here

        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
