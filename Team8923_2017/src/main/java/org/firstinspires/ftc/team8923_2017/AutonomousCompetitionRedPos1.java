package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Autonomous Competition Red 1", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutonomousCompetitionRedPos1 extends MasterAutonomous
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
        closeGG();
        sleep(1000);
        moveGG(1500);
        sleep(500);
        DropJJ();
        sleep(1000);
        GetLeftJewelColorCount();
        //CameraDevice.getInstance().setFlashTorchMode(false);
        double referenceAngle =  imu.getAngularOrientation().firstAngle;

        if (isLeftJewelRed == true)
        {
            IMUPivot(referenceAngle, -15, 0.5, 0.015);
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);
        }
        else
        {
            IMUPivot(referenceAngle, 15, 0.5, 0.015);
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);
        }
        sleep(500);
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 2.2);//Go towards parking spot//Was 2.15
        sleep(500);
        MoveIMURight(referenceAngle, 45.0, 0.0, 0.015, 0.35, 0.2);//Moves away from the cryptobox and towards the middle//Was0.4
        sleep(500);
        IMUPivot(referenceAngle, 90, 0.25, 0.015);//Pivots to face the cryptobox
        referenceAngle += 90.0;//Sets angle to 0
        referenceAngle = adjustAngles(referenceAngle);
        sleep(1000);
        moveGG(-750);
        sleep(1000);
        alignOnLine55(0.5, 5.0, 0.29);//Aligns on line
        //MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.8);
        sleep(500);
        if (vuMark == RelicRecoveryVuMark.LEFT)
        {
            MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.3);
            MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.68);
            telemetry.addData("Stage", "Left");
            telemetry.update();
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER)
        {
            MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.05);
            MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
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
        sleep(500);
        openGG();
        sleep(500);
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.6);
        sleep(500);
        MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.7);
        sleep(500);
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.6);
        if (vuMark == RelicRecoveryVuMark.LEFT)
        {
            MoveIMURight(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.68);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.3);
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER)
        {
            MoveIMURight(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.05);
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
        {
            MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.55);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.3);
        }
        else
        {
            MoveIMURight(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
            MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.05);
            telemetry.addData("Stage", "Else");
            telemetry.update();
            //MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
        }

        while (opModeIsActive())
        {
            idle();
        }

    }
}
