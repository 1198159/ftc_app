package org.firstinspires.ftc.team8923_2017;

import android.annotation.TargetApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Autonomous Blue 1", group = "Swerve")
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


        waitForStart();
        GetVumark();
        // turn on flash light
        //CameraDevice.getInstance().setFlashTorchMode(true);
        // set false to turn off light
        sleep(700);
        closeGG();
        sleep(700);
        moveGG(1500);
        DropJJ();
        sleep(500);
        stopGG();
        GetLeftJewelColorCount();
        //CameraDevice.getInstance().setFlashTorchMode(false);
        double referenceAngle =  imu.getAngularOrientation().firstAngle;

        if (isLeftJewelRed == true)
        {
            IMUPivot(referenceAngle, 15, 0.5, 0.015);
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);
        }
        else
        {
            IMUPivot(referenceAngle, -15, 0.5, 0.015);
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);
        }
        sleep(700);
        MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 2.15);//Go towards parking spot//Was 2.01 for new pos
        MoveIMURight(referenceAngle, 45.0, 0.0, 0.015, 0.35, 0.1); // Was 0.4
        IMUPivot(referenceAngle, 90, 0.25, 0.015);
        referenceAngle += 90.0;
        referenceAngle = adjustAngles(referenceAngle);
        sleep(500);
        moveGG(-750);
        sleep(700);
        stopGG();
        sleep(500);
        alignOnLine55(0.4, 3.0, 0.2);
        sleep(500);
        if (vuMark == RelicRecoveryVuMark.LEFT)
        {
            MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.3);
            MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.58);
            telemetry.addData("Stage", "Left");
            telemetry.update();
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER)
        {
            MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.05);
            //MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
            telemetry.addData("Stage", "Center");
            telemetry.update();
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
        {
            MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.3);
            MoveIMURight(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.55);
            telemetry.addData("Stage", "Right");
            telemetry.update();
        }
        else
        {
            MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.05);
            MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
            //MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
        }
        sleep(500);
        MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.8);
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.1);
        sleep(700);
        moveGG(-750);
        sleep(700);
        stopGG();
        sleep(500);
        openGG();
        sleep(500);
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.55, 0.22);
        sleep(500);
        MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 1.5);
        sleep(500);
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.45, 0.5);
        if (vuMark == RelicRecoveryVuMark.LEFT)
        {
            MoveIMURight(referenceAngle, 190.0, 0.0, 0.015, 0.55, 0.43);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.55, 0.12);
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER)
        {
            MoveIMURight(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.05);
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
        {
            MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.55, 0.35);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.55, 0.2);
        }
        else
        {
            MoveIMURight(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.05);
            telemetry.addData("Stage", "Else");
            telemetry.update();
            //MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
        }
        IMUPivot(referenceAngle,  -178, 0.45, 0.015);
        stopDriving();
        /*
        sleep(1000);
        MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.8);
        sleep(500);
        openGG();
        sleep(500);
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.3);
        sleep(500);
        moveGG(-750);
        sleep(500);
        */

        /*
        MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.6);
        sleep(500);
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.6);
        */
        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
