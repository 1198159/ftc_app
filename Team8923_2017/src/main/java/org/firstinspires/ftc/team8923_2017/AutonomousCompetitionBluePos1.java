package org.firstinspires.ftc.team8923_2017;

import android.annotation.TargetApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

@Autonomous(name="Autonomous Competition Blue Pos 1", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
@TargetApi(21)
public class AutonomousCompetitionBluePos1 extends MasterAutonomous
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
        GetLeftJewelColor();
        CameraDevice.getInstance().setFlashTorchMode(false);
        double referenceAngle =  imu.getAngularOrientation().firstAngle;

        if (isLeftJewelRed == true)
        {
            IMUPivot(referenceAngle, 20, 0.5, 0.015);
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);
        }
        else
        {
            IMUPivot(referenceAngle, -20, 0.5, 0.015);
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);
        }
        MoveIMU(referenceAngle, 50.0, 0.0, 0.015, 0.5, 3.0);//Go towards parking spot
        IMUPivot(referenceAngle, -90, 0.5, 0.015);
        referenceAngle -= 90.0;
        referenceAngle = adjustAngles(referenceAngle);
        MoveIMU(referenceAngle, -50.0, 0.0, 0.015, 0.5, 0.8);
        moveGG(-1500);
        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}