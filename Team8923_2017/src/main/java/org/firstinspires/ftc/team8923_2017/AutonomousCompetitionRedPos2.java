package org.firstinspires.ftc.team8923_2017;

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Autonomous Red 2", group = "Swerve")
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

        telemetry.addData("InitState:", "InitStarted");
        telemetry.update();
        InitAuto();//Initializes Hardware and sets position based on alliance
        initVuforia();//Initializes Vuforia
        telemetry.addData("InitState:", "InitFinished");
        telemetry.update();


        waitForStart();
        RetrieveJJ();
        motorFF.setTargetPosition(motorFF.getCurrentPosition() + 5);
        motorFF.setPower((motorFF.getTargetPosition() - motorFF.getCurrentPosition()) * (1 / 100.0));
        // turn on flash light
        CameraDevice.getInstance().setFlashTorchMode(true);
        GetVumark();
        // set false to turn off light
        closeGG();
        sleep(700);
        moveGG(1500);
        DropJJ();
        sleep(500);
        stopGG();
        //turnOnFlash(4000);
        GetLeftJewelColorCount();
        CameraDevice.getInstance().setFlashTorchMode(false);
        double referenceAngle =  imu.getAngularOrientation().firstAngle;

        if (isLeftJewelRed == true)
        {
            IMUPivot(referenceAngle, -15, 0.5, 0.015);//Pivot right
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);//Pivot left
        }
        else
        {
            IMUPivot(referenceAngle, 15, 0.5, 0.015);
            RetrieveJJ();
            IMUPivot(referenceAngle, 0, 0.5, 0.015);
        }
        sleep(700);
        MoveIMU(referenceAngle, -900.0, 0.0, 0.015, 0.3, 1.57);//Go towards parking spot
        IMUPivot(referenceAngle, -90, 0.25, 0.015);
        referenceAngle -= 90.0;
        referenceAngle = adjustAngles(referenceAngle);
        MoveIMU(referenceAngle, 900.0, 0.0, 0.015, 0.3, 1.3);//aWas 1.2 for middle
        IMUPivot(referenceAngle, -88, 0.25, 0.015);
        referenceAngle -= 90.0;
        referenceAngle = adjustAngles(referenceAngle);
        sleep(500);
        moveGG(-750);
        sleep(700);
        stopGG();
        sleep(500);
        MoveIMU(referenceAngle, -900, 0.0, 0.015, 0.3, 0.4);
        alignOnLine(0.5, 3.0, 0.3);
        sleep(500);
        if (vuMark == RelicRecoveryVuMark.LEFT)
        {
            MoveIMU(referenceAngle, -900.0, 0.0, 0.015, 0.35, 0.3);
            MoveIMULeft(referenceAngle, 900.0, 0.0, 0.015, 0.35, 0.67);
            telemetry.addData("Stage", "Left");
            telemetry.update();
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER)
        {
            MoveIMU(referenceAngle, -900.0, 0.0, 0.015, 0.35, 0.05);
            MoveIMULeft(referenceAngle, 900.0, 0.0, 0.015, 0.35, 0.16);
            telemetry.addData("Stage", "Center");
            telemetry.update();
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
        {
            MoveIMU(referenceAngle, -900.0, 0.0, 0.015, 0.35, 0.46);
            MoveIMURight(referenceAngle, 900.0, 0.0, 0.015, 0.35, 0.52);
            telemetry.addData("Stage", "Right");
            telemetry.update();
        }
        else
        {
            MoveIMU(referenceAngle, -900.0, 0.0, 0.015, 0.35, 0.05);
            MoveIMULeft(referenceAngle, 900.0, 0.0, 0.015, 0.35, 0.1);
            //MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
        }
        sleep(500);
        MoveIMU(referenceAngle, 900.0, 0.0, 0.015, 0.35, 0.8);
        MoveIMU(referenceAngle, -900.0, 0.0, 0.015, 0.35, 0.1);
        sleep(700);
        moveGG(-750);
        sleep(700);
        stopGG();
        sleep(500);
        openGG();
        sleep(500);
        MoveIMU(referenceAngle, -900.0, 0.0, 0.015, 0.55, 0.22);
        //sleep(500);
        MoveIMU(referenceAngle, 900.0, 0.0, 0.015, 0.35, 1.5);
        //sleep(500);
        MoveIMU(referenceAngle, -900.0, 0.0, 0.015, 0.45, 0.5);

        if (vuMark == RelicRecoveryVuMark.LEFT)
        {
            MoveIMURight(referenceAngle, 900.0, 0.0, 0.015, 0.55, 0.43);
            MoveIMU(referenceAngle, 900.0, 0.0, 0.015, 0.55, 0.12);
        }
        else if (vuMark == RelicRecoveryVuMark.CENTER)
        {
            MoveIMURight(referenceAngle, 900.0, 0.0, 0.015, 0.35, 0.1);
            MoveIMU(referenceAngle, 900.0, 0.0, 0.015, 0.35, 0.05);
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT)
        {
            MoveIMULeft(referenceAngle, 900.0, 0.0, 0.015, 0.55, 0.35);
            MoveIMU(referenceAngle, 900.0, 0.0, 0.015, 0.55, 0.1);
        }
        else
        {
            MoveIMURight(referenceAngle, 900.0, 0.0, 0.015, 0.35, 0.1);
            MoveIMU(referenceAngle, 900.0, 0.0, 0.015, 0.35, 0.05);
            telemetry.addData("Stage", "Else");
            telemetry.update();
            //MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);
        }
        IMUPivot(referenceAngle,  125.0, 0.45, 0.015);
        stopDriving();
        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
