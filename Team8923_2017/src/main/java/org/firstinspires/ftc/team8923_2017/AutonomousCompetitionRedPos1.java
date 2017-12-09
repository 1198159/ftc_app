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
        GetVumark();

        waitForStart();
        // turn on flash light
        //CameraDevice.getInstance().setFlashTorchMode(true);
        // set false to turn off light
        closeGG();
        sleep(500);
        moveGG(1500);
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
        sleep(700);
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 2.26);//Go towards parking spot//Was 2.15
        MoveIMURight(referenceAngle, 45.0, 0.0, 0.015, 0.35, 0.2);//Moves away from the cryptobox and towards the middle//Was0.4
        IMUPivot(referenceAngle, 90, 0.25, 0.015);//Pivots to face the cryptobox
        referenceAngle += 90.0;//Sets angle to 0
        referenceAngle = adjustAngles(referenceAngle);
        sleep(500);
        moveGG(-750);
        sleep(500);
        alignOnLine55(0.5, 3.0, 0.29);//Aligns on line
        //MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.8);
        sleep(500);
        //if (vuMark == RelicRecoveryVuMark.LEFT)
        //MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.3);
        //MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.7);

        //if (vuMark == RelicRecoveryVuMark.CENTER)
        MoveIMULeft(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.1);


        /*
        MoveIMU(referenceAngle, 190.0, 0.0, 0.015, 0.35, 0.6);
        sleep(500);
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 0.6);
        */
        while (opModeIsActive())
        {
            telemetry.addData("EncoderFL", motorFL.getCurrentPosition());
            telemetry.addData("EncoderFR", motorFR.getCurrentPosition());
            telemetry.addData("EncoderBL", motorBL.getCurrentPosition());
            telemetry.addData("EncoderBR", motorBR.getCurrentPosition());
            telemetry.update();
            //Run();
            idle();
        }

    }
}
