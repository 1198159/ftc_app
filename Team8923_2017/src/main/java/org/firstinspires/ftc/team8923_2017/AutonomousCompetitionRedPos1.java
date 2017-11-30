package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

@Autonomous(name="Autonomous Competition Red Pos 1", group = "Swerve")
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
        //Was -50 mm
        /*
        MoveIMU(referenceAngle, -90.0, 0.0, 0.015, 0.5, 1.5);//Go towards parking spot
        MoveIMURight(referenceAngle, 45.0, 0.0, 0.015, 0.35, 0.4);
        IMUPivot(referenceAngle, 90, 0.25, 0.015);
        referenceAngle += 90.0;
        referenceAngle = adjustAngles(referenceAngle);
        //MoveIMU(referenceAngle, 40.0, 0.0, 0.015, 0.5, 0.8);
        moveGG(-1500);
        */
        sleep(700);
        MoveIMU(referenceAngle, -190.0, 0.0, 0.015, 0.35, 2.15);//Go towards parking spot//Was1.65
        MoveIMURight(referenceAngle, 45.0, 0.0, 0.015, 0.35, 0.4);
        IMUPivot(referenceAngle, 90, 0.25, 0.015);
        referenceAngle += 90.0;
        referenceAngle = adjustAngles(referenceAngle);
        //MoveIMU(referenceAngle, 40.0, 0.0, 0.015, 0.5, 0.8);
        moveGG(-1500);


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
